# Debug Probe Usage Notes

## Priority: Debug Probe Over LED Debugging

**CRITICAL GUIDELINE: Always prioritize debug probe over LED-based debugging**, especially when encountering issues that prevent output (USB enumeration failures, hardfaults, early crashes).

### Why Debug Probe First

1. **Always works** - Even when USB is completely broken, the probe can flash, halt, and inspect the device
2. **Full visibility** - GDB gives you register state, memory contents, and stack traces that LEDs never can
3. **Faster iteration** - Flash via probe in seconds vs manual BOOTSEL dance
4. **Catches what LEDs miss** - Hardfaults before LED init, crashes between LED states

### When to Consider LED Debugging

Only use LED-based debugging when:
- Debug probe is not physically connected
- You specifically need to observe runtime behavior without breakpoints
- Testing production firmware that won't have probe attached

Even then, prefer adding diagnostic output over LED patterns when possible.

---

## Two OpenOCD Installations (CRITICAL)

There are TWO OpenOCD versions installed on this system:

| Version | Path | RP2350 Support |
|---------|------|----------------|
| **System (Chocolatey)** | `C:/ProgramData/chocolatey/lib/openocd/` | **NO** (0.12.0 from 2023) |
| **Pico SDK** | `/c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/` | **YES** (0.12.0+dev from 2025) |

**ALWAYS use full paths** to ensure the correct OpenOCD version is used. The system `openocd` in PATH does NOT have RP2350 support.

---

## Reliable OpenOCD Startup (USE THIS)

**Standard command to start OpenOCD - use this every time:**

```bash
# Kill any existing OpenOCD, wait for USB release, then start fresh
taskkill //F //IM openocd.exe 2>/dev/null; sleep 2; /c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/openocd -s /c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/scripts -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" &
```

This command is idempotent - safe to run even if no OpenOCD is running.

**Why this works:**
1. `taskkill //F //IM openocd.exe` - Kills ALL OpenOCD processes (pkill doesn't work on Windows)
2. `sleep 2` - Allows USB device to be released
3. Full paths ensure Pico SDK OpenOCD is used (not system version)
4. `&` runs in background so terminal remains usable

**When done debugging:**
```bash
taskkill //F //IM openocd.exe
```

---

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

---

## Working Commands

### Start OpenOCD (RP2350) - FULL PATH VERSION
```bash
# Always kill first, then start with full paths
taskkill //F //IM openocd.exe 2>/dev/null; sleep 2; /c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/openocd -s /c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/scripts -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" &
```

### GDB Connection (single line - multiline doesn't work well in bash)
```bash
cd /c/Users/pow-w/Documents/Rocket-Chip && arm-none-eabi-gdb build/rocketchip.elf -batch -ex "target extended-remote localhost:3333" -ex "monitor reset halt" -ex "bt"
```

### Flash and Run via GDB
```bash
cd /c/Users/pow-w/Documents/Rocket-Chip && arm-none-eabi-gdb build/rocketchip.elf -batch -ex "target extended-remote localhost:3333" -ex "monitor reset halt" -ex "load" -ex "monitor reset run"
```

---

## Known Issues

1. **Multiline bash commands**: The `\` line continuation doesn't work reliably with GDB. Use single-line commands or `&&` chaining.

2. **Timeouts**: GDB batch mode can hang waiting for breakpoints. Use short timeout values.

3. **OpenOCD version confusion**: System OpenOCD (Chocolatey, 0.12.0) doesn't have rp2350.cfg. **Always use full paths** to Pico SDK's OpenOCD at `/c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/`.

4. **Target state**: After GDB disconnects, may need `monitor reset halt` to regain control.

5. **USB I/O Errors with PC=0x00000000**: When you see "error submitting USB write: Input/Output Error" and PC=0x00000000 with corrupt stack, the device is in a bad state. Try:
   - Kill OpenOCD completely: `taskkill //F //IM openocd.exe`
   - Wait 2-3 seconds
   - Restart OpenOCD with full path command
   - Use `monitor reset halt` before any other commands
   - If still failing, power cycle the device (unplug USB) or use picotool to flash

6. **Dual USB conflict**: The RP2350 target's USB CDC and the debug probe are separate USB connections to the host. However, if the target firmware crashes with USB in a bad state, the CMSIS-DAP probe may also report I/O errors. Power cycling the target usually resolves this.

7. **"Unable to find CMSIS-DAP device"** - **RESOLVED**

   **Root cause identified (2026-01-30):** Multiple stale OpenOCD processes holding the USB device exclusively.

   **Solution:** Always kill all OpenOCD processes before starting:
   ```bash
   taskkill //F //IM openocd.exe 2>/dev/null; sleep 2; <start openocd with full path>
   ```

   **Why `pkill openocd` didn't work:** The `pkill` command is not available in Windows Git Bash. Use `taskkill //F //IM openocd.exe` instead.

---

## Iterative Debugging Tips

### NEVER Add Debug Printf - Use Probe Instead

**CRITICAL:** When debugging issues, **do NOT add printf/debug statements** as the first approach. Printf can:
1. Change timing and hide/create race conditions
2. Block on USB CDC mutex, causing deadlocks
3. Affect the very issue you're trying to debug

**Always use the debug probe first:**
```bash
# Halt and get backtrace
arm-none-eabi-gdb build/rocketchip.elf -batch -ex "target extended-remote localhost:3333" -ex "monitor halt" -ex "bt"

# Check specific variables
arm-none-eabi-gdb build/rocketchip.elf -batch -ex "target extended-remote localhost:3333" -ex "monitor halt" -ex "print my_variable"

# Check which task is running on each core
arm-none-eabi-gdb build/rocketchip.elf -batch -ex "target extended-remote localhost:3333" -ex "monitor halt" -ex "print (char*)pxCurrentTCBs[0]->pcTaskName" -ex "print (char*)pxCurrentTCBs[1]->pcTaskName"
```

The probe gives you instant visibility without modifying code behavior.

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
cd /c/Users/pow-w/Documents/Rocket-Chip && arm-none-eabi-gdb build/rocketchip.elf -batch -ex "target extended-remote localhost:3333" -ex "monitor reset halt" -ex "load" -ex "monitor reset run"
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
