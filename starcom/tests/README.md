# Tests (`tests/`)

Host-side only. Procedure: [`../docs/TESTING.md`](../docs/TESTING.md). Named vectors: [`../docs/IVP.md`](../docs/IVP.md).

```
cmake -S starcom -B starcom/build -G Ninja
cmake --build starcom/build
ctest --test-dir starcom/build
```

`starcom.unit` is the host binary: increment 0+1 codecs, increment 2 FOP-P/FARM-P (`test_copp.cpp`), increment 3 USLP (`test_uslp.cpp`). Truncated USLP / FECF / Insert Zone are not this sitting.
