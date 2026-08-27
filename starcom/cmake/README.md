# CMake modules (`cmake/`)

| File | Job |
|------|-----|
| `CompilerWarnings.cmake` | C++20; core `-fno-exceptions -fno-rtti`; test flags |
| `starcom_version.cmake` | `STARCOM_VERSION` + git → `generated/starcom/version.hpp` |

Root `CMakeLists.txt` builds static `starcom`, alias `Starcom::starcom`. Host `ctest` via `tests/`. Not wired into the Pico firmware CMake. `tl::expected` is vendored under `third_party/`.
