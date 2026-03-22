# Project Memory

<!-- This file is the live session memory for Claude. Keep it concise.
     Use it for: pointers to knowledge/ files, framework-level gotchas not tied to a specific component.
     Do not duplicate detail that belongs in knowledge/ files. -->

## Hardware: Ai-Thinker ESP32-Audio-Kit (ESP32-A1S V2.3, ES8388)

Full config → `/workspace/knowledge/esp32-audio-kit-a1s.md`

Key quirks:
- Chip is **ESP32** (LX6), NOT ESP32-S3 — target is `esp32`, bootloader at 0x1000
- Serial via USB-UART bridge (CP2102/CH340) on `/dev/ttyESP` — standard `idf.py flash` works (DTR/RTS reset)
- MCLK must be GPIO0 for ES8388 — shared with BOOT button
- MIC2 and LINEIN are shared — cannot be used simultaneously
- Mount SD card before I2S init (GPIO25/26 conflict risk)

## Project Setup

- Toolchain / runtime: ESP-IDF v5.5.3 at `/opt/esp/esp-idf`
- Target / platform: `esp32`
- Main source: `/workspace/src/hello-world` (confirmed working)
- Build: `bash -c "source /opt/esp/esp-idf/export.sh && idf.py build"` (must chain in one shell)
- Flash: standard esptool `--before default_reset`, bootloader at **0x1000**
- Monitor: Python serial reader (115200 baud) — idf.py monitor not reliable in non-TTY env
