# Tests (`tests/`)

Host-side only. Procedure: [`../docs/TESTING.md`](../docs/TESTING.md). Named vectors: [`../docs/IVP.md`](../docs/IVP.md).

```
cmake -S starcom -B starcom/build -G Ninja
cmake --build starcom/build
ctest --test-dir starcom/build
```

`starcom.unit` is the host binary: codecs, COP-P (`test_copp.cpp`), USLP (`test_uslp.cpp`), COP-1 (`test_cop1.cpp`). FOP-1 S4/S5 BC-init is not this sitting.
