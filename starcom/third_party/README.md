# Vendored third-party

Checked in so configure does not FetchContent these on every run.

| Path | What | License |
|------|------|---------|
| `tl/expected.hpp` | [TartanLlama/expected](https://github.com/TartanLlama/expected) v1.1.0. Default backend for `starcom::ccsds::Result<T>`. | CC0-1.0 (`tl/COPYING`) |

Public compile also needs this directory on the include path: `Result` is `tl::expected`. Knob `STARCOM_USE_STD_EXPECTED` (later) swaps to `std::expected`.
