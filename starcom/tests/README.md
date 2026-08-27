# Tests (`tests/`)

Host-side only. Procedure: [`../docs/TESTING.md`](../docs/TESTING.md). Named vectors: [`../docs/IVP.md`](../docs/IVP.md).

```
cmake -S starcom -B starcom/build -G Ninja
cmake --build starcom/build
ctest --test-dir starcom/build
```

`starcom.unit` is the host binary: increment 0+1 codec goldens plus increment 2 FOP-P/FARM-P tables (`test_copp.cpp`). SET V(R) persistent activity (211.0 7.2.3.2) is not in this sitting.
