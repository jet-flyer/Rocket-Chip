# Tests (`tests/`)

Host-side only. Procedure: [`../docs/TESTING.md`](../docs/TESTING.md). Named vectors: [`../docs/IVP.md`](../docs/IVP.md).

```
cmake -S starcom -B starcom/build -G Ninja
cmake --build starcom/build
ctest --test-dir starcom/build
```

`starcom.unit` is the host binary: codecs, COP-P, USLP, COP-1, loopback, version, prefix smoke (`test_fuzz.cpp`). `-DSTARCOM_SANITIZE=ON` when libasan is present.
