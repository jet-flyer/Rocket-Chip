# Benchmark Task: UD Factorization + DCP Float64 Investigation

## Context

RocketChip runs a 24-state ESKF on RP2350 (dual Cortex-M33 @ 150MHz, 520KB SRAM, 8MB PSRAM). The current implementation uses float32 throughout with:
- **Codegen FPFT** for covariance propagation (`predict()`) — SymPy-generated flat scalar C++ with CSE, benchmarked at 59µs avg (IVP-47)
- **Joseph-form scalar Kalman update** (`scalar_kalman_update()`) — O(N²) rank-1 update with explicit symmetry enforcement
- **Dense Mat24 (24×24 float32 array)** for P storage — `float data[24][24]` = 2,304 bytes

The RP2350 has:
- **Hardware FPU**: Single-cycle float32 multiply/add
- **DCP (Double-precision Coprocessor)**: Custom Pi-designed coprocessor, ~2-3 cycles per float64 basic op. Not a full FPU — data interchange between CPU and DCP via `mrrc`/`mcrr` instructions. Pico SDK automatically uses DCP when you use `double` type in C.
- **RISC-V cores CANNOT access the DCP** — ARM Cortex-M33 only

We want to evaluate whether UD factorization with mixed-precision (float64 accumulators in critical inner loops) is worth implementing. This benchmark should provide the same kind of concrete data that the IVP-47 dense-vs-sparse-vs-codegen experiment provided.

## What To Benchmark

### Test 1: DCP Raw Throughput Characterization
Measure actual cycle counts for the operations that matter in UD factorization:

```
float32 multiply-accumulate (FPU): a += b * c
float64 multiply-accumulate (DCP): a += b * c  
float32 → float64 promotion + accumulate: double_acc += (double)f32_a * (double)f32_b
float64 → float32 demotion: f32_result = (float)f64_value
```

Run each in a tight loop (1000 iterations, prevent optimizer removal), measure with `time_us_32()`. Report cycles per op. This tells us the real DCP overhead on our specific board.

### Test 2: Current P Propagation Precision Analysis
Using the existing `predict()` (codegen FPFT) and `predict_dense()` (reference), run a stress test that exposes float32 weaknesses:

1. Initialize ESKF with realistic state (use values from a typical stationary soak)
2. Run 10,000 consecutive predict cycles at dt=0.001 (simulating 10 seconds of 1kHz)
3. After each step, check:
   - `P.min_diagonal()` — any negative values? (symmetry/PD violation)
   - `P.max_diagonal()` — any runaway growth?
   - `P.is_symmetric(tol)` — max |P(i,j) - P(j,i)| before `force_symmetric()`
4. Report: how many steps before issues appear, magnitude of asymmetry accumulation

This quantifies "how broken is float32 currently" — if P stays healthy for 10,000 steps, UD is less urgent. If it degrades after 1,000, UD is critical.

### Test 3: UD Factorization Implementation + Benchmark

Implement a minimal UD factorization for the 24×24 P matrix:

**Storage:**
```cpp
struct UD24 {
    float U[24][24];  // Unit upper triangular (only upper triangle used)
    float D[24];      // Diagonal
};
// Total: 2,400 bytes (vs 2,304 for dense P — negligible difference)
```

**Thornton temporal update** (P_new = F·P·F^T + G·Q·G^T in UD form):
- Input: Current U, D, state transition effects (from codegen), process noise Q diagonal
- Output: Updated U, D
- Key inner loop: accumulation of `U[i][k] * D[k] * U[j][k]` across k
- Implement THREE variants:
  a. **Pure float32**: all accumulations in float32
  b. **Mixed precision**: inner dot-product accumulator is `double`, stored result is `float`
  c. **Pure float64 accumulation**: promote all arithmetic in the inner loop to double

**Bierman scalar measurement update** (replaces current Joseph-form `scalar_kalman_update`):
- Input: U, D, measurement H row (sparse — single nonzero for baro/GPS), innovation, R
- Output: Updated U, D, Kalman gain vector
- Implement float32 only (measurement update is numerically benign)

Benchmark all three Thornton variants + the Bierman update against the current codegen predict + Joseph update.

### Test 4: Numerical Accuracy Comparison
Run the same 10,000-step stress test from Test 2, but now with:
- **Path A**: Current codegen FPFT + Joseph update (float32)
- **Path B**: UD Thornton (float32) + Bierman update
- **Path C**: UD Thornton (mixed f32/f64 accumulator) + Bierman update

After each step, convert UD back to dense P (P = U·D·U^T) and compare:
- Max diagonal difference between paths
- Is P from each path still positive definite? (check all D[i] > 0 for UD paths)
- Frobenius norm of difference between Path A and Path C
- At what step count do the paths diverge significantly?

Also inject a pathological scenario: set one position state uncertainty to 1e6 (simulating long GPS-denied drift) while bias uncertainties remain at 1e-8. This maximizes the condition number of P and stress-tests numerical stability.

### Test 5: Full Cycle Timing
Time a complete ESKF cycle as it would run in flight:
```
1x predict (codegen or UD Thornton)
1x baro scalar update  
3x mag scalar updates
6x GPS scalar updates (when available)
```

Compare total cycle time for:
- Current architecture (codegen FPFT + Joseph scalar updates)
- UD architecture (Thornton + Bierman scalar updates), float32
- UD architecture (Thornton + Bierman scalar updates), mixed precision

## Implementation Notes

### Using the DCP
The Pico SDK handles DCP transparently. Just use `double` type:
```cpp
// This automatically uses DCP on ARM RP2350
double accumulator = 0.0;
for (int k = 0; k < N; ++k) {
    accumulator += (double)U[i][k] * (double)D[k] * (double)U[j][k];
}
result[i][j] = (float)accumulator;  // demote back for storage
```

**CRITICAL**: Use `1.0` not `1.0f` for double literals. GCC defaults literals to double, but be explicit. Conversely, make sure float32 paths use `1.0f` everywhere to avoid accidental double promotion.

### Timing
Use `time_us_32()` (microsecond timer) wrapping the operation under test. For sub-microsecond operations, use the SysTick cycle counter:
```cpp
// Cycle-accurate timing
volatile uint32_t start = systick_hw->cvr;
// ... operation ...
volatile uint32_t end = systick_hw->cvr;
uint32_t cycles = (start - end) & 0xFFFFFF; // SysTick counts DOWN, 24-bit
```

### SRAM Placement
Put the UD arrays in `.time_critical` SRAM section (same as codegen FPFT) to avoid PSRAM cache penalties:
```cpp
static UD24 __attribute__((section(".time_critical"))) g_ud;
```

### Reference Algorithms

**Thornton WMGS temporal update** — pseudocode from Ramos et al. (arXiv:2203.06105):
```
Input: U, D (current), Phi (state transition), G (noise input), Q (process noise diagonal)
Output: U_new, D_new

W = [Phi*U | G]           // n×(n+q) matrix
D_bar = diag([D | Q])     // (n+q)×(n+q) diagonal

For j = n down to 1:
    d_j = sum_k(W[j][k]^2 * D_bar[k])  // <-- THIS is the critical accumulation
    D_new[j] = d_j
    For i = 1 to j-1:
        c = sum_k(W[i][k] * D_bar[k] * W[j][k])  // <-- and THIS
        U_new[i][j] = c / d_j
        For k:
            W[i][k] -= U_new[i][j] * W[j][k]
```

**Bierman scalar measurement update** — pseudocode:
```
Input: U, D, h (measurement row), R (scalar noise), innovation
Output: Updated U, D, gain K

// Forward pass
f = U^T * h
g = D .* f
alpha[0] = R + f[0]*g[0]
K[0] = g[0]

For j = 1 to n-1:
    alpha[j] = alpha[j-1] + f[j]*g[j]
    lambda = -f[j] / alpha[j-1]
    D[j] = D[j] * alpha[j-1] / alpha[j]
    For i = 0 to j-1:
        U_save = U[i][j]
        U[i][j] = U_save + lambda * K[i]
        K[i] = K[i] + g[j] * U_save
        
K = K / alpha[n-1]  // final gain

// State update: x = x + K * innovation
```

### Output Format
Print results over USB serial in a format easy to parse:
```
=== UD FACTORIZATION BENCHMARK ===
Board: RP2350 HSTX Feather @ 150MHz

--- Test 1: DCP Throughput ---
f32 MAC: XX.X cycles/op
f64 MAC (DCP): XX.X cycles/op
f32→f64 promoted MAC: XX.X cycles/op
f64→f32 demotion: XX.X cycles/op
DCP overhead ratio: X.Xx

--- Test 2: Current P Stability ---
Steps before negative diagonal: XXXX (or "none in 10000")
Max asymmetry at step 1000: X.XXe-XX
Max asymmetry at step 5000: X.XXe-XX
Max asymmetry at step 10000: X.XXe-XX
Min diagonal at step 10000: X.XXe-XX

--- Test 3: Implementation Timing ---
Thornton f32: XXX µs
Thornton mixed f32/f64: XXX µs
Thornton f64 accum: XXX µs
Bierman scalar update: XX µs
Current codegen FPFT: XX µs (reference)
Current Joseph scalar: XX µs (reference)

--- Test 4: Numerical Accuracy ---
[table of path comparisons at steps 1000, 5000, 10000]
Pathological condition test: [results]

--- Test 5: Full Cycle ---
Current (codegen+Joseph): XXX µs
UD f32 (Thornton+Bierman): XXX µs  
UD mixed (Thornton+Bierman): XXX µs
```

## Decision Criteria
After this benchmark, we'll know:
1. **Is float32 P actually breaking?** If Test 2 shows no issues in 10,000 steps, UD is a correctness insurance policy, not a fix for an active problem.
2. **What does DCP actually cost?** If mixed precision is <2× float32, it's an easy win. If >5×, it's only worth using in the tightest inner loops.
3. **Does UD Thornton fit in the cycle budget?** Current codegen FPFT is 59µs. If UD Thornton is <200µs, it fits easily at 1kHz. If >500µs, we need the codegen approach for UD too (possible but more work).
4. **Does mixed precision actually improve anything measurable?** Test 4 shows whether the DCP's extra precision produces meaningfully different P evolution compared to pure float32 UD.

## Files to Create
- `src/fusion/ud_factor.h` — UD24 struct + function declarations
- `src/fusion/ud_factor.cpp` — Thornton + Bierman implementations (3 Thornton variants)
- `src/benchmark/ud_benchmark.cpp` — All 5 tests
- Build as a standalone benchmark binary (like previous IVP-47 benchmark)

## Repo Reference
- Current ESKF: `src/fusion/eskf.h`, `src/fusion/eskf.cpp`
- Codegen FPFT: `src/fusion/eskf_codegen.h` (generated by `scripts/generate_fpft.py`)
- Matrix types: `src/math/mat.h` — `Mat<R,C>` template, float32
- Previous benchmark pattern: IVP-47 (dense 538µs → codegen 59µs, 9.1× speedup)
