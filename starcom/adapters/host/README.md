# Host adapters

Desktop/ground transports. CMake target `Starcom::adapters_host` depends on `Starcom::starcom`. Core never depends on this target.

This sitting: `loopback.cpp` (`FrameSlot` / `HostLoopback`). `RadioPort` is header-only on the same slot. UDP and file replay wait.
