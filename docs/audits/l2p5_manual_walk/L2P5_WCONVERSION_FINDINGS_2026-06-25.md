# L2-P5 — `-Wconversion` / `-Wsign-conversion` located findings (§CM mechanical remediation batch)

**Generated 2026-06-25** by syntax-checking every authored `src/` TU (`build/compile_commands.json`) with
`-Wsign-conversion -Wconversion` (warnings, not errors). **Class 14 is demoted to mechanical (§CM/§SC), not a
semantic walk** — these are a **§CM mechanical remediation batch** (the tool found them; no human needed to
*locate* them). Disposition each: real narrowing/wrap bug → **fix**; intentional → **explicit cast (+ why)**;
mandated `uintN_t`/HW/bitmask (JSF-163 exemption) → **leave**. Do NOT blind-`static_cast` all of them — that would
mask a real bug. **fix-then-gate** (can't `-Werror` a dirty tree); `eskf*` hits need bit-exact verification.

**54 findings across 12 files.** `eskf*` hits need bit-exact verification on fix.

| File:line | Flag | Message |
|---|---|---|
| `src/active_objects/ao_radio.cpp:322` | `-Wsign-conversion` | conversion to 'int8_t' {aka 'signed char'} from 'uint8_t' {aka 'unsigned char'} may change |
| `src/calibration/calibration_manager.cpp:299` | `-Wsign-conversion` | unsigned conversion from 'int' to 'long unsigned int' changes value from '-3' to '42949672 |
| `src/calibration/calibration_manager.cpp:673` | `-Wsign-conversion` | unsigned conversion from 'int' to 'long unsigned int' changes value from '-2' to '42949672 |
| `src/calibration/calibration_manager.cpp:741` | `-Wconversion` | conversion from 'int' to 'uint8_t' {aka 'unsigned char'} may change value |
| `src/calibration/calibration_manager.cpp:771` | `-Wconversion` | conversion from 'int' to 'uint8_t' {aka 'unsigned char'} may change value |
| `src/calibration/calibration_manager.cpp:967` | `-Wconversion` | conversion from 'int' to 'uint16_t' {aka 'short unsigned int'} may change value |
| `src/cli/rc_os_commands.cpp:456` | `-Wconversion` | conversion from 'uint32_t' {aka 'long unsigned int'} to 'uint16_t' {aka 'short unsigned in |
| `src/cli/rc_os_dashboard.cpp:221` | `-Wsign-conversion` | conversion to 'int' from 'size_t' {aka 'unsigned int'} may change the sign of the result |
| `src/cli/rc_os_dashboard.cpp:228` | `-Wsign-conversion` | conversion to 'int' from 'size_t' {aka 'unsigned int'} may change the sign of the result |
| `src/diag/diag_stats.cpp:52` | `-Wsign-conversion` | conversion to 'uint' {aka 'unsigned int'} from 'int' may change the sign of the result |
| `src/drivers/gps_pa1010d.cpp:318` | `-Wsign-conversion` | conversion to 'size_t' {aka 'unsigned int'} from 'int' may change the sign of the result |
| `src/drivers/icm20948.cpp:136` | `-Wfloat-conversion` | conversion from 'double' to 'float' changes value from '1.3323124061025417e-4' to '1.33231 |
| `src/drivers/icm20948.cpp:137` | `-Wfloat-conversion` | conversion from 'double' to 'float' changes value from '2.6646248122050834e-4' to '2.66462 |
| `src/drivers/icm20948.cpp:138` | `-Wfloat-conversion` | conversion from 'double' to 'float' changes value from '5.3211258920466417e-4' to '5.32112 |
| `src/drivers/icm20948.cpp:139` | `-Wfloat-conversion` | conversion from 'double' to 'float' changes value from '1.0642251784093283e-3' to '1.06422 |
| `src/drivers/icm20948.cpp:220` | `-Wsign-conversion` | unsigned conversion from 'int' to 'uint8_t' {aka 'unsigned char'} changes the value of '-3 |
| `src/drivers/icm20948.cpp:457` | `-Wsign-conversion` | unsigned conversion from 'int' to 'uint8_t' {aka 'unsigned char'} changes the value of '-3 |
| `src/drivers/ws2812_status.cpp:113` | `-Wconversion` | conversion from 'int' to 'uint8_t' {aka 'unsigned char'} may change value |
| `src/drivers/ws2812_status.cpp:114` | `-Wconversion` | conversion from 'int' to 'uint8_t' {aka 'unsigned char'} may change value |
| `src/drivers/ws2812_status.cpp:115` | `-Wconversion` | conversion from 'int' to 'uint8_t' {aka 'unsigned char'} may change value |
| `src/drivers/ws2812_status.cpp:144` | `-Wconversion` | conversion from 'int' to 'uint8_t' {aka 'unsigned char'} may change value |
| `src/drivers/ws2812_status.cpp:145` | `-Wconversion` | conversion from 'int' to 'uint8_t' {aka 'unsigned char'} may change value |
| `src/drivers/ws2812_status.cpp:146` | `-Wconversion` | conversion from 'int' to 'uint8_t' {aka 'unsigned char'} may change value |
| `src/fusion/eskf_codegen.cpp:348` | `-Wfloat-conversion` | conversion from 'double' to 'float' may change value |
| `src/fusion/eskf_codegen.cpp:509` | `-Wfloat-conversion` | conversion from 'double' to 'float' may change value |
| `src/fusion/eskf_codegen.cpp:545` | `-Wfloat-conversion` | conversion from 'double' to 'float' may change value |
| `src/fusion/eskf_codegen.cpp:546` | `-Wfloat-conversion` | conversion from 'double' to 'float' may change value |
| `src/fusion/eskf_codegen.cpp:547` | `-Wfloat-conversion` | conversion from 'double' to 'float' may change value |
| `src/fusion/eskf_codegen.cpp:548` | `-Wfloat-conversion` | conversion from 'double' to 'float' may change value |
| `src/fusion/eskf_codegen.cpp:549` | `-Wfloat-conversion` | conversion from 'double' to 'float' may change value |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/math/mat.h:19` | `-Wsign-conversion` | conversion to 'unsigned int' from 'long int' may change the sign of the result |
| `src/safety/pio_backup_timer.cpp:52` | `-Wsign-conversion` | conversion to 'uint32_t' {aka 'long unsigned int'} from 'int' may change the sign of the r |
| `src/safety/pio_watchdog.cpp:35` | `-Wsign-conversion` | conversion to 'uint32_t' {aka 'long unsigned int'} from 'int' may change the sign of the r |

### Per-file counts

- 22  `src/math/mat.h`
- 7  `src/fusion/eskf_codegen.cpp`
- 6  `src/drivers/icm20948.cpp`
- 6  `src/drivers/ws2812_status.cpp`
- 5  `src/calibration/calibration_manager.cpp`
- 2  `src/cli/rc_os_dashboard.cpp`
- 1  `src/active_objects/ao_radio.cpp`
- 1  `src/cli/rc_os_commands.cpp`
- 1  `src/diag/diag_stats.cpp`
- 1  `src/drivers/gps_pa1010d.cpp`
- 1  `src/safety/pio_backup_timer.cpp`
- 1  `src/safety/pio_watchdog.cpp`
