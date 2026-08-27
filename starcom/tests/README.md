# Tests (`tests/`)

Host-side only. Procedure: [`../docs/TESTING.md`](../docs/TESTING.md). Named vectors: [`../docs/IVP.md`](../docs/IVP.md).

```
cmake -S starcom -B starcom/build -G Ninja
cmake --build starcom/build
ctest --test-dir starcom/build
```

`starcom.unit` is the increment 0+1 codec binary (PLTU/CRC goldens, IVP rejects, D-5 heap trap). Table-driven FOP-P/FARM-P comes later.
