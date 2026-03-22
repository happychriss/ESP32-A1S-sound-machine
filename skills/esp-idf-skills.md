---
name: esp32-debugging
description: Debug ESP32 firmware issues including compilation errors, runtime panics, memory issues, and communication failures
---

# ESP32 Firmware Debugging Guide

## When to Use This Skill

Apply this skill when the user:
- Encounters compilation errors in ESP-IDF projects
- Sees runtime panics or "Guru Meditation Error" messages
- Has memory-related crashes or stack overflows
- Experiences I2C/SPI/UART communication failures
- Needs help interpreting serial monitor output

## Debugging Techniques

### 1. Compilation Error Analysis

**Missing Includes**
```
fatal error: driver/gpio.h: No such file or directory
```
Fix: Check CMakeLists.txt and add the component to REQUIRES:
```cmake
idf_component_register(
    SRCS "main.c"
    REQUIRES driver
)
```

**Undefined References**
```
undefined reference to 'some_function'
```
Fix: Ensure the component containing the function is in REQUIRES or PRIV_REQUIRES.

**Type Errors**
Look for mismatched types between function declarations and implementations.

### 2. Runtime Panic Analysis

**Guru Meditation Error Patterns**

| Error | Cause | Fix |
|-------|-------|-----|
| `StoreProhibited` | Writing to invalid memory | Check pointer initialization |
| `LoadProhibited` | Reading from invalid memory | Check null pointers |
| `InstrFetchProhibited` | Corrupted function pointer | Check callback assignments |
| `IntegerDivideByZero` | Division by zero | Add zero checks |

**Stack Overflow**
```
Guru Meditation Error: Core 0 panic'ed (Stack overflow)
```
Fix: Increase stack size in task creation:
```c
xTaskCreatePinnedToCore(task_fn, "name", 4096, NULL, 5, NULL, 0);
//                                        ^^^^ increase this
```

**Stack Smashing**
```
Stack smashing detected
```
Fix: Local buffer overflow - check array bounds and string operations.

### 3. Memory Debugging

**Check Heap Usage**
```c
ESP_LOGI(TAG, "Free heap: %lu", esp_get_free_heap_size());
ESP_LOGI(TAG, "Min free heap: %lu", esp_get_minimum_free_heap_size());
```

**Common Memory Issues**
- Memory leak: Missing `free()` after `malloc()`
- Double free: Freeing same memory twice
- Use after free: Accessing freed memory

### 4. Communication Debugging

**I2C Issues**
```
E (1234) i2c: i2c_master_cmd_begin(xxx): I2C_NUM error
```
Checklist:
- Verify I2C address (7-bit vs 8-bit format)
- Check SDA/SCL GPIO pins
- Ensure pull-up resistors are present (4.7K typical)
- Verify clock frequency compatibility

**Serial/UART Issues**
- Baud rate mismatch
- TX/RX swapped
- Missing ground connection

### 5. Build Commands for Debugging

```bash
# Clean build to eliminate stale objects
make robocar-clean && make robocar-build-main

# Build with verbose output
cd packages/esp32-projects/robocar-main && idf.py build -v

# Start serial monitor
make robocar-monitor-main PORT=/dev/cu.usbserial-0001
```

### 6. Useful ESP-IDF Config Options

Enable in `sdkconfig` or via `idf.py menuconfig`:
- `CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT` - Print panic info before reboot
- `CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK` - Detect stack overflow earlier
- `CONFIG_HEAP_POISONING_COMPREHENSIVE` - Detect heap corruption

---

## 7. Deep Sleep Current Budget — ESP32-S3R8 Reality (confirmed via research)

### Two unavoidable sources that dominate measured current

**Source 1: USB host connected → BBPLL stays alive**

`CONFIG_RTC_CLOCK_BBPLL_POWER_ON_WITH_USB=y` is the ESP-IDF default. When a PC
(USB host) is connected during deep sleep, the 480 MHz BBPLL cannot shut down.
This adds **~1–2 mA** and makes all firmware-level power optimisations appear useless
because this one source drowns everything else out.

Symptom: peripheral power optimisations (display SLPIN, GPIO holds, IMU off) make
no measurable difference — all stages read ~2 mA regardless.

Fix for measurement: **disconnect USB, power from LiPo only.**
Fix in firmware (optional, breaks USB reconnect after wakeup):
```
CONFIG_RTC_CLOCK_BBPLL_POWER_ON_WITH_USB=n   # in sdkconfig.defaults
```
Note: connecting to a USB charger (not a PC) does NOT trigger BBPLL — only a live
USB host enumeration does.

**Source 2: ESP32-S3R8 octal PSRAM — permanently on with EXT1/GPIO wakeup**

The 8 MB embedded PSRAM shares VDD_SPI with the SPI flash. With any GPIO/EXT1
wakeup source active, ESP-IDF cannot power down VDD_SPI during deep sleep.
Result: PSRAM draws **~140 µA** at all times (from ESP32-S3-WROOM-1 datasheet Table 6-7).

VDD_SPI CAN be powered down with timer-only wakeup:
```c
esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
```
But this is incompatible with EXT1/IMU wakeup.

### Realistic deep sleep floor — Waveshare ESP32-S3-Touch-LCD-1.9

| Configuration | Measured/Estimated | Notes |
|---------------|--------------------|-------|
| USB host (PC) connected | **~2 mA** | BBPLL dominates |
| LiPo J1 connector (no USB) | **~2 mA** | ETA6098 BAT→SYS quiescent dominates |
| 3.3V rail direct, EXT1 (IMU) wakeup | **~220 µA** | PSRAM 140 + IMU 55 + ESP32 8 + buck 11 + LCD 7 |
| 3.3V rail direct, timer + VDDSDIO off | **~80 µA** | No PSRAM, no IMU |

**ETA6098 charger IC — critical distinction:**
The "1 µA at BAT" spec is the *shutdown* mode quiescent (VIN=0, CE=0).
When the IC is actively providing power from LiPo (BAT→SYS, no USB VIN), its
internal control circuitry draws **~1–2 mA** at the BAT pin. This is confirmed
by `min_sleep_test`: all 4 stages (baseline → LCD SLPIN → touch reset → IMU WoM)
measured identically at ~2 mA from the J1 JST connector. This floor is NOT
firmware-controllable.

**To measure actual MCU/peripheral sleep current:** bypass the ETA6098 by
supplying the 3.3V rail directly (via test pad), or use a LiPo battery without
routing through a meter at the charger input.

QMI8658A WoM at 128 Hz Low-Power: **55 µA** — negligible.
MP1605GTF-Z buck Iq: **11 µA** — negligible.

---

## 8. Deep Sleep GPIO Hold — Confirmed Patterns (ESP32-S3, ESP-IDF v5.x)

### The problem: `CONFIG_ESP_SLEEP_GPIO_RESET_WORKAROUND`

The default sdkconfig includes `CONFIG_ESP_SLEEP_GPIO_RESET_WORKAROUND=y`. This calls
`esp_sleep_config_gpio_isolate()` at boot, which floats **all digital GPIO pads** during
deep sleep — including pads you set before calling `esp_deep_sleep_start()`. Visible in
boot log as:

```
sleep_gpio: Configure to isolate all GPIO pins in sleep state
```

`gpio_deep_sleep_hold_en()` alone is **not sufficient** to hold digital GPIO output levels
when this workaround is active. The isolation overrides the hold for digital pads.

---

### Correct pattern: use RTC domain for pins that must hold during sleep

For any output GPIO that must retain its level during deep sleep:

```c
// Move pin to RTC domain — immune to digital GPIO isolation workaround
rtc_gpio_init(GPIO_NUM);
rtc_gpio_set_direction(GPIO_NUM, RTC_GPIO_MODE_OUTPUT_ONLY);
rtc_gpio_set_level(GPIO_NUM, level);
rtc_gpio_hold_en(GPIO_NUM);   // pin-specific, persists through deep sleep
```

Do this for each pin individually. Do **not** use `gpio_deep_sleep_hold_en()` when you
also have `rtc_gpio_hold_en()` calls — see the EXT1 wakeup hazard below.

---

### Hazard: `gpio_deep_sleep_hold_en()` + `rtc_gpio_hold_en()` blocks EXT1 wakeup

`gpio_deep_sleep_hold_en()` sets a flag that causes `esp_deep_sleep_start()` to call
`gpio_hold_en()` on **all** GPIO pads at sleep entry. For RTC-capable GPIOs (GPIO0–GPIO21
on ESP32-S3), `gpio_hold_en()` delegates to `rtc_gpio_hold_en()`. This holds the EXT1
wakeup pin in its current (LOW) state and prevents the RTC controller from detecting the
wakeup edge.

**Symptom:** device enters deep sleep but can never be woken by EXT1/IMU motion.

**Fix:** do not use `gpio_deep_sleep_hold_en()`. Use `rtc_gpio_hold_en()` only for the
specific pins that need holding. The EXT1 wakeup pin (e.g. GPIO8) must not be held.

---

### Hazard: `rtc_gpio_hold_en()` persists across deep sleep wakeup

Unlike `gpio_deep_sleep_hold_en()`, `rtc_gpio_hold_en()` is a **hardware latch in the
RTC IO MUX** that survives deep sleep and remains active after wakeup. If a peripheral
reset pin (e.g. CST816S touch on GPIO17) is held LOW, the peripheral driver's reset
release on boot will silently fail → `ESP_ERROR_CHECK` panic → crash/reboot loop.

**Fix:** at the start of `app_main()`, always release holds and return pins to digital domain:

```c
rtc_gpio_hold_dis(HELD_GPIO_A);
rtc_gpio_deinit(HELD_GPIO_A);
rtc_gpio_hold_dis(HELD_GPIO_B);
rtc_gpio_deinit(HELD_GPIO_B);
```

Safe to call on cold boot even if no hold was previously set.

---

### Correct deep sleep sequence (confirmed working, ESP32-S3)

```c
// 1. Immediate output change via digital GPIO (fast, before RTC domain switch)
gpio_set_level(OUTPUT_PIN, level);

// 2. Do all I2C / SPI configuration while still in digital domain

// 3. Configure EXT1 wakeup pin as RTC input
rtc_gpio_init(WAKEUP_PIN);
rtc_gpio_set_direction(WAKEUP_PIN, RTC_GPIO_MODE_INPUT_ONLY);
rtc_gpio_pulldown_en(WAKEUP_PIN);
rtc_gpio_pullup_dis(WAKEUP_PIN);
esp_sleep_enable_ext1_wakeup(1ULL << WAKEUP_PIN, ESP_EXT1_WAKEUP_ANY_HIGH);

// 4. Move output pins to RTC domain — AFTER EXT1 config, AFTER I2C is done
rtc_gpio_init(OUTPUT_PIN);
rtc_gpio_set_direction(OUTPUT_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
rtc_gpio_set_level(OUTPUT_PIN, level);
rtc_gpio_hold_en(OUTPUT_PIN);   // holds through sleep; release at next boot

// 5. Enter sleep — NO gpio_deep_sleep_hold_en()
esp_deep_sleep_start();
```

---

## Examples

### Example: Debugging a Stack Overflow

User reports: "My ESP32 keeps crashing on startup"

1. Ask for serial monitor output
2. Look for "Stack overflow" in panic message
3. Identify which task is overflowing
4. Suggest increasing stack size from 2048 to 4096
5. Explain FreeRTOS stack sizing considerations

### Example: I2C Communication Failure

User reports: "I2C device not responding"

1. Verify address with I2C scanner
2. Check GPIO configuration
3. Verify pull-up resistors
4. Check bus speed compatibility
5. Suggest adding delays between transactions if needed
