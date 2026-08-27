# CMake modules (`cmake/`)

Install/export helpers, warnings, sanitizer toggles. Empty until the first codec `.cpp` creates `Starcom::starcom`.

Shape (already locked in ICD / IVP, not a new fork): static `starcom`, alias `Starcom::starcom`, C++20, core `-fno-exceptions -fno-rtti`, host `ctest`, not wired into the Pico firmware CMake in the first sitting. `tl::expected` vendor path is ICD. This directory holds the helper `.cmake` files when that sitting lands.
