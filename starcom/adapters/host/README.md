# Host adapters

Desktop/ground transports. CMake target `Starcom::adapters_host` depends on `Starcom::starcom`. Core never depends on this target.

This sitting: `loopback.cpp` (`FrameSlot` / `HostLoopback`), `file_replay.cpp` (`replayPltuFile`), `udp.cpp` (`udpBind` / `udpSendTo` / `udpRecv` / `udpClose`), with the OS calls in `udp_win32.cpp` or `udp_posix.cpp` (CMake builds one of the two). `RadioPort` is header-only on the same slot. Caller owns host, path, and port; bind port `0` is the OS ephemeral port — there is no Starcom service port. Socket I/O lives only in these `.cpp` files. RP2350 SPI glue is increment 16.
