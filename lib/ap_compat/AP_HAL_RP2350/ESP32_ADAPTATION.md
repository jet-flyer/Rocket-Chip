# ESP32 HAL Adaptation Tracker for RP2350

This document tracks what can be reused from ArduPilot's AP_HAL_ESP32 for our RP2350 port.

**Status Legend:**
- `VERBATIM` - Can be used as-is or with trivial namespace changes
- `ADAPT` - Needs platform-specific replacements (documented below)
- `REPLACE` - Significantly different, use as reference only
- `SKIP` - Not needed for RP2350

---

## Global Platform Differences

| ESP32 (ESP-IDF) | RP2350 (Pico SDK) | Notes |
|-----------------|-------------------|-------|
| `esp_rom_delay_us(us)` | `busy_wait_us(us)` | Microsecond busy-wait |
| `esp_restart()` | `watchdog_reboot(0,0,0)` | System reboot |
| `esp_task_wdt_*` | `watchdog_enable/update()` | Task watchdog |
| `xTaskCreatePinnedToCore(..core)` | `xTaskCreateAffinitySet()` | Core affinity (SMP) |
| `IRAM_ATTR` | `__not_in_flash_func()` | RAM-resident functions |
| `heap_caps_*` | Standard malloc or `pvPortMalloc` | Heap allocation |
| `esp_timer_get_time()` | `time_us_64()` | Microsecond timer |
| `esp_efuse_mac_get_*` | `pico_unique_board_id_t` | Unique device ID |
| `esp_partition_*` | `flash_range_*` | Flash storage |
| `portYIELD_FROM_ISR_ARG(x)` | `portYIELD_FROM_ISR()` | ISR yield |
| `CONFIG_FREERTOS_HZ` | `configTICK_RATE_HZ` | FreeRTOS tick rate |

---

## HAL_ESP32_Class.h / HAL_ESP32_Class.cpp

**Status:** ADAPT (template for our HAL class structure)

### Verbatim Patterns
- Static driver instances at file scope
- HAL constructor passing pointers to base class `AP_HAL::HAL(...)`
- `run()` method setting callbacks and calling `scheduler->init()`
- Use of `Empty::` namespace for unimplemented drivers
- `AP_HAL::init()` empty stub

### Adaptations Needed
| ESP32 | RP2350 | Notes |
|-------|--------|-------|
| `ESP32::` namespace | `RP2350::` namespace | Namespace change |
| `HAL_ESP32_WIFI` conditionals | Remove | No WiFi driver |
| `AP_HAL_Empty/` includes | Keep for unused drivers | Same pattern |
| Serial driver count | Adjust for RP2350 UART count | 2 UARTs + USB CDC |

### Files to Reference
```cpp
// Static instance pattern - KEEP
static RP2350::UARTDriver cons(0);
static RP2350::Scheduler schedulerInstance;
static RP2350::Storage storageDriver;
// etc.

// Constructor pattern - KEEP
HAL_RP2350::HAL_RP2350() :
    AP_HAL::HAL(
        &cons,           // serial0/console
        // ... all pointers
        nullptr          // CAN ifaces
    )
{}

// Run pattern - KEEP
void HAL_RP2350::run(int argc, char * const argv[], Callbacks* callbacks) const {
    ((RP2350::Scheduler *)hal.scheduler)->set_callbacks(callbacks);
    hal.scheduler->init();
}
```

---

## Scheduler.h / Scheduler.cpp

**Status:** ADAPT (~80% reusable)

### Verbatim Code
- Class structure inheriting from `AP_HAL::Scheduler`
- `register_timer_process()` / `register_io_process()` pattern
- `_run_timers()` / `_run_io()` callback execution pattern
- Timer/IO semaphore protection
- `in_main_thread()` using `xTaskGetCurrentTaskHandle()`
- `delay()` implementation calling `delay_microseconds()` in loop
- `thread_create()` with priority mapping
- `_initialized` flag pattern
- Task entry point static functions

### Adaptations Needed
| ESP32 Code | RP2350 Equivalent | Location |
|------------|-------------------|----------|
| `esp_task_wdt_init(&config)` | `watchdog_enable(timeout_ms, true)` | `init()`, `_main_thread()` |
| `esp_task_wdt_reset()` | `watchdog_update()` | `_main_thread()` loop |
| `esp_task_wdt_add(NULL)` | Not needed | Remove |
| `esp_rom_delay_us(us)` | `busy_wait_us(us)` | `delay_microseconds()` |
| `xTaskCreatePinnedToCore(...FASTCPU)` | `xTaskCreateAffinitySet()` or `xTaskCreate()` | `init()` |
| `esp_restart()` | `watchdog_reboot(0,0,0)` | `reboot()` |
| `IRAM_ATTR` | `__not_in_flash_func()` | Function attributes |
| `hal.rcout->force_safety_on()` | Remove or stub | `reboot()` |
| `unmount_sdcard()` | Remove | `reboot()` |
| `hal.analogin->_timer_tick()` | Keep if using AnalogIn | `_timer_thread()` |
| `hal.rcout->timer_tick()` | Remove initially | `_rcout_thread()` |
| `((RCInput*)hal.rcin)->_timer_tick()` | Remove initially | `_rcin_thread()` |
| `mount_sdcard()` / `sdcard_retry()` | Remove | `_io_thread()` |
| `hal.util->get_soft_armed()` | Keep | Arm state check |

### Tasks to Create
| Task | Priority | Core | Notes |
|------|----------|------|-------|
| APM_MAIN | High (24) | Core 0 | Main loop, watchdog |
| APM_TIMER | High (23) | Core 0 | 1kHz timer callbacks |
| APM_UART | High (23) | Core 0 | Serial I/O |
| APM_IO | Low (5) | Core 1 | Background I/O |
| APM_STORAGE | Low (4) | Core 1 | Flash writes |

### Code to Remove
- `_rcout_thread` - No RC output initially
- `_rcin_thread` - No RC input initially
- SD card mounting code
- `print_stats()` / `print_main_loop_rate()` - ESP32 specific debug
- `vTaskGetRunTimeStats()` / `heap_caps_print_heap_info()` - ESP32 heap debug

---

## Semaphores.h / Semaphores.cpp

**Status:** VERBATIM (~95% reusable)

### Verbatim Code
- `Semaphore` class using `xSemaphoreCreateRecursiveMutex()`
- `give()` using `xSemaphoreGiveRecursive()`
- `take()` with timeout loop
- `take_blocking()` using `portMAX_DELAY`
- `take_nonblocking()`
- `check_owner()` using `xSemaphoreGetMutexHolder()`
- `BinarySemaphore` class using `xSemaphoreCreateBinary()`
- `wait()` / `wait_blocking()` / `signal()`

### Single Adaptation
| ESP32 Code | RP2350 Equivalent | Location |
|------------|-------------------|----------|
| `portYIELD_FROM_ISR_ARG(xHigherPriorityTaskWoken)` | `portYIELD_FROM_ISR()` | `signal_ISR()` |

---

## Storage.h / Storage.cpp

**Status:** ADAPT (~70% reusable, flash API differs)

### Verbatim Code
- Class structure inheriting from `AP_HAL::Storage`
- `AP_FlashStorage` integration pattern
- `_buffer[]` RAM cache
- `_dirty_mask` bitmask for write tracking
- `read_block()` / `write_block()` from buffer
- `_timer_tick()` writing dirty lines
- `_flash_load()` / `_flash_write()` pattern
- `healthy()` check based on last empty time
- `get_storage_ptr()` returning buffer

### Adaptations Needed
| ESP32 Code | RP2350 Equivalent | Location |
|------------|-------------------|----------|
| `esp_partition_find_first(...)` | Direct flash offset calculation | `_storage_open()` |
| `esp_partition_write(p, addr, data, len)` | `flash_range_program(offset, data, len)` | `_flash_write_data()` |
| `esp_partition_read(p, addr, data, len)` | Direct XIP read: `memcpy(data, (void*)(XIP_BASE + offset), len)` | `_flash_read_data()` |
| `esp_partition_erase_range(p, addr, size)` | `flash_range_erase(offset, len)` | `_flash_erase_sector()` |
| `const esp_partition_t *p` member | `uint32_t _flash_offset` | Class member |
| `STORAGE_SECTOR_SIZE (128*1024)` | `FLASH_SECTOR_SIZE (4096)` | RP2350 has 4KB sectors |

### RP2350-Specific Requirements (per PD1 in RP2350_FULL_AP_PORT.md)
- Use `__not_in_flash_func()` on flash operation wrappers
- Call `save_and_disable_interrupts()` / `restore_interrupts()` around flash ops
- Call `xip_cache_invalidate_all()` after flash operations
- Page-align writes to 256 bytes for `flash_range_program()`

---

## Util.h / Util.cpp

**Status:** ADAPT (~60% reusable)

### Verbatim Code
- Class structure inheriting from `AP_HAL::Util`
- `safety_switch_state()` returning `SAFETY_NONE`
- `was_watchdog_reset()` pattern
- `thread_info()` stub

### Adaptations Needed
| ESP32 Code | RP2350 Equivalent | Location |
|------------|-------------------|----------|
| `heap_caps_get_largest_free_block()` | `xPortGetFreeHeapSize()` | `available_memory()` |
| `heap_caps_calloc(1, size, MALLOC_CAP_*)` | `pvPortMalloc(size)` + `memset()` | `malloc_type()` |
| `heap_caps_free(ptr)` | `vPortFree(ptr)` | `free_type()` |
| `esp_efuse_mac_get_*` | `pico_get_unique_board_id()` | `get_system_id()` |
| `esp_timer_get_time()` | `time_us_64()` | `get_hw_rtc()` |
| `esp_reset_reason()` | `watchdog_caused_reboot()` | `was_watchdog_reset()` |
| `HAL_ESP32_BOARD_NAME` | `"RocketChip"` | Board name string |
| Tone alarm PWM code | Remove or adapt | Optional feature |

---

## UARTDriver.h / UARTDriver.cpp

**Status:** REPLACE (ESP-IDF UART API too different)

### Patterns to Keep
- Class structure inheriting from `AP_HAL::UARTDriver`
- `ByteBuffer _readbuf` / `_writebuf` ring buffers
- `Semaphore _write_mutex` for thread safety
- `_timer_tick()` for background I/O
- `_receive_timestamp` for timing
- Protected methods: `_begin()`, `_end()`, `_flush()`, `_available()`, `_read()`, `_write()`

### Must Replace
| ESP32 Code | RP2350 Equivalent | Notes |
|------------|-------------------|-------|
| `driver/uart.h` | `hardware/uart.h` | Different API |
| `uart_port_t` | `uart_inst_t*` (uart0/uart1) | RP2350 has 2 UARTs |
| `uart_driver_install()` | `uart_init()` | Init pattern |
| `uart_read_bytes()` | `uart_read_blocking()` or FIFO check | Read |
| `uart_write_bytes()` | `uart_write_blocking()` or FIFO | Write |
| GPIO num config | `gpio_set_function(pin, GPIO_FUNC_UART)` | Pin mux |

### USB CDC Consideration
RP2350 typically uses USB CDC for console (serial0). ESP32 uses UART0.
- May need separate `USBSerialDriver` class or TinyUSB integration
- Pico SDK's `stdio_usb` handles this if using `printf`

---

## I2CDevice.h / I2CDevice.cpp

**Status:** REPLACE (ESP-IDF I2C API differs significantly)

### Patterns to Keep
- `I2CDevice` inheriting from `AP_HAL::I2CDevice`
- `I2CDeviceManager` managing device instances
- Semaphore for bus locking
- `transfer()` method signature

### Must Replace
| ESP32 Code | RP2350 Equivalent |
|------------|-------------------|
| `driver/i2c.h` | `hardware/i2c.h` |
| `i2c_master_*` API | `i2c_write_blocking()` / `i2c_read_blocking()` |
| `i2c_cmd_handle_t` command queuing | Direct blocking calls |

---

## SPIDevice.h / SPIDevice.cpp

**Status:** REPLACE (ESP-IDF SPI API differs)

### Patterns to Keep
- `SPIDevice` inheriting from `AP_HAL::SPIDevice`
- `SPIDeviceManager` with device descriptors
- `transfer()` method signature
- Chip select handling pattern

### Must Replace
| ESP32 Code | RP2350 Equivalent |
|------------|-------------------|
| `driver/spi_master.h` | `hardware/spi.h` |
| `spi_device_handle_t` | `spi_inst_t*` (spi0/spi1) |
| `spi_device_transmit()` | `spi_write_read_blocking()` |
| DMA configuration | RP2350 DMA channels |

---

## GPIO.h / GPIO.cpp

**Status:** ADAPT (~70% reusable)

### Verbatim Code
- Class inheriting from `AP_HAL::GPIO`
- `pinMode()` / `read()` / `write()` / `toggle()` signatures

### Adaptations Needed
| ESP32 Code | RP2350 Equivalent |
|------------|-------------------|
| `gpio_set_direction()` | `gpio_set_dir()` |
| `gpio_set_level()` | `gpio_put()` |
| `gpio_get_level()` | `gpio_get()` |
| `gpio_set_pull_mode()` | `gpio_pull_up()` / `gpio_pull_down()` |

---

## AnalogIn.h / AnalogIn.cpp

**Status:** ADAPT

### Patterns to Keep
- Class inheriting from `AP_HAL::AnalogIn`
- `AnalogSource` per-channel class
- `_timer_tick()` for sampling
- Voltage scaling

### Adaptations Needed
| ESP32 Code | RP2350 Equivalent |
|------------|-------------------|
| `adc1_get_raw()` | `adc_read()` |
| ADC attenuation config | RP2350 doesn't need this |
| ADC channel mapping | GPIO26-29 = ADC0-3 |

---

## Files to Skip

| File | Reason |
|------|--------|
| `WiFiDriver.h/cpp` | No WiFi on RP2350 |
| `WiFiUdpDriver.h/cpp` | No WiFi |
| `RCInput.h/cpp` | Not needed initially |
| `RCOutput.h/cpp` | Not needed initially |
| `SdCard.h/cpp` | Using flash, not SD |
| `Profile.h/cpp` | ESP32-specific profiling |
| `RmtSigReader.h` | ESP32 RMT peripheral |
| `SoftSigReader*.h` | ESP32-specific |
| `i2c_sw.h` | Software I2C fallback |

---

## Implementation Order

1. **Semaphores** - Nearly verbatim, foundation for others
2. **Scheduler** - Core functionality, FreeRTOS based
3. **Util** - Simple adaptations
4. **Storage** - We already have this working, verify it matches pattern
5. **HAL_RP2350_Class** - Wire everything together
6. **GPIO** - Simple adaptations
7. **UARTDriver** - Platform-specific but well-defined
8. **I2CDevice** - After UART works
9. **SPIDevice** - After I2C works
10. **AnalogIn** - Lower priority

---

## Verification Checklist

For each file adapted:
- [ ] Compiles without errors
- [ ] Inherits from correct `AP_HAL::` base class
- [ ] All pure virtual methods implemented with `override`
- [ ] Uses `RP2350::` namespace
- [ ] No ESP-IDF includes remaining
- [ ] Platform-specific code uses Pico SDK equivalents
- [ ] Smoke test passes

---

## Notes

- ESP32 HAL uses `AP_HAL_Empty` namespace for stub drivers - we should do the same
- ESP32 pins tasks to cores (FASTCPU=0, SLOWCPU=1) - RP2350 can do this with FreeRTOS SMP
- ESP32 uses `IRAM_ATTR` extensively - map to `__not_in_flash_func()` for flash-critical code
- Both use FreeRTOS, so task/semaphore patterns are nearly identical
