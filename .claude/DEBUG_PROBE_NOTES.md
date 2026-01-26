# Debug Probe Usage Notes

## Issue: USB CDC breaks after flash operations (RESOLVED)

**Root Cause:**
Per RP2350_FULL_AP_PORT.md PD1+PD3:
- Flash operations make entire flash inaccessible during erase/write
- TinyUSB interrupt handlers are in flash
- When flash ops run with USB active, USB IRQ handlers can't execute → USB breaks

**Solution (correct approach per DEBUG_OUTPUT.md):**
Do flash operations BEFORE USB is fully active:
1. stdio_init_all() - enables USB driver
2. hal.init() / storage.init() - flash operations happen here
3. THEN wait for USB connection
4. THEN print output

This follows DEBUG_OUTPUT.md pattern: "Run program logic immediately, wait for connection before printing results."

**Previous failed workarounds (for reference):**
- sleep_ms() delays after flash ops - partial TX fix, RX still broken
- recoverUsbInput() draining - didn't help
- Extended recovery times - didn't help

The issue was architectural - USB must not be active during flash operations.

---

## Working Commands

### Start OpenOCD (RP2350)
```bash
OPENOCD_DIR=/c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/
${OPENOCD_DIR}openocd -s ${OPENOCD_DIR}scripts -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000"
```

### GDB Connection (single line - multiline doesn't work well in bash)
```bash
cd /c/Users/pow-w/Documents/Rocket-Chip && arm-none-eabi-gdb -batch -ex "target extended-remote localhost:3333" -ex "monitor reset halt" -ex "file build/smoke_calibration.elf" -ex "load" -ex "info breakpoints" build/smoke_calibration.elf
```

## Known Issues

1. **Multiline bash commands**: The `\` line continuation doesn't work reliably with GDB. Use single-line commands or `&&` chaining.

2. **Timeouts**: GDB batch mode can hang waiting for breakpoints. Use short timeout values.

3. **OpenOCD version**: System OpenOCD (0.12.0) doesn't have rp2350.cfg. Must use pico-sdk's OpenOCD at `~/.pico-sdk/openocd/0.12.0+dev/`.

4. **Target state**: After GDB disconnects, may need `monitor reset halt` to regain control.

## Iterative Debugging Tips

### Add Version String to Debug Output
When iteratively debugging crashes, add a version identifier to confirm the correct binary is running:

```cpp
printf("Build: v3-static-calibrator\n");  // Update version on each change
```

This prevents confusion about whether the latest code was actually flashed.

### Use Debug Probe Before Asking for Manual BOOTSEL
When USB is unresponsive:
1. First try flashing via debug probe (always works if probe connected)
2. Only ask for manual BOOTSEL as last resort

```bash
arm-none-eabi-gdb firmware.elf -batch \
  -ex "target extended-remote localhost:3333" \
  -ex "monitor reset halt" \
  -ex "load" \
  -ex "monitor reset run"
```

---

## Useful GDB Commands

- `monitor reset halt` - Reset and halt target
- `load` - Load ELF to flash
- `break file.cpp:line` - Set breakpoint
- `continue` - Run until breakpoint
- `next` / `step` - Step over/into
- `print variable` - Print variable value
- `bt` - Backtrace
- `info locals` - Show local variables

## Ports
- GDB: 3333
- Telnet: 4444
- TCL: 6666
