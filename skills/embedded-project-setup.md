---
name: embedded-project-setup
description: Set up working conventions for an embedded firmware project — folder structure, knowledge flow, and build workflow
---

# Embedded Project Setup

## 1. Folder Structure

Ensure these folders exist at the workspace root:

```
/workspace/
├── requirements.md  # ROOT — project type pointer + functional specification for all features
├── external-docs/   # Raw external material — datasheets, vendor docs, board wikis (read-only)
├── knowledge/        # Curated driver notes — confirmed working config derived from docs + experiment
├── skills/          # Reusable Claude skills (this file lives here)
└── src/             # Firmware source projects
```

Create any missing folders. Do not create a README in `docs/` or `knowledge/` — the files are self-describing.

---

## 2. Knowledge Flow

This is the rule for where hardware knowledge lives:

```
docs/            →    knowledge/           →    memory.md
(vendor source)       (confirmed config)       (thin pointers only)
```

### external-docs/
- Drop zone for external material: PDFs, vendor markdown, wiki exports
- Never edit these files
- When the user adds a file here, analyse it and create or update the corresponding `knowledge/` file

### knowledge/
- One `.md` file per board or component (e.g. `waveshare-esp32s3-touch-lcd-1.9.md`)
- Contains: pin tables, confirmed driver config, experimentally validated quirks
- Structure each peripheral as:
  ```
  ## Peripheral Name — Chip (confirmed working)
  | Signal | GPIO | Notes |
  ...
  ### ESP-IDF Driver
  - confirmed config values
  ```
- **Write back here** whenever experimental work confirms or corrects a value (pin polarity, timing, flags, etc.)
- This is the single source of truth — not memory.md

### memory.md
- Stays concise: pointers to `knowledge/` files + framework-level gotchas that aren't hardware facts
- Never duplicate detail that belongs in `knowledge/`
- Format:
  ```
  ## Hardware: <Board Name>
  Full config → /workspace/knowledge/<file>.md
  Key quirks: ...
  ```

---

## 3. Analysing a New Doc

When the user adds a file to `external-docs/`:

1. Read it fully
2. Extract: chip name, interface, pins, timing, any init sequence quirks
3. Create `knowledge/<component>.md` with a pin table + known config
4. Mark unconfirmed values clearly: `# unconfirmed — from datasheet`
5. Tell the user what was captured and what needs experimental validation

---

## 4. ESP-IDF Build Conventions

### Environment
- ESP-IDF v5.4.1 at `/opt/esp/esp-idf`
- Activate: `source /opt/esp/esp-idf/export.sh`
- Target: `esp32s3`

### New Project Checklist
- [ ] `main/idf_component.yml` — declare registry dependencies
- [ ] `sdkconfig.defaults` — baseline Kconfig overrides (fonts, tick rate, etc.)
- [ ] `main/CMakeLists.txt` REQUIRES — use `log` not `esp_log`
- [ ] Clean before target change: `rm -rf build managed_components sdkconfig`

### Flashing
`idf.py flash` fails on USB-CDC — always use esptool with `--before=usb_reset`:
```bash
source /opt/esp/esp-idf/export.sh
python3 -m esptool --chip esp32s3 -p /dev/ttyESP -b 460800 \
  --before=usb_reset --after=hard_reset write_flash \
  --flash_mode dio --flash_freq 80m --flash_size detect \
  0x0  build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x10000 build/<project>.bin
```

### Serial Monitor
`idf.py monitor` unreliable on USB-CDC — use Python serial with DTR/RTS reset:
```python
import serial, time, sys
s = serial.Serial('/dev/ttyESP', 115200, timeout=1)
s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False
end = time.time() + 15
while time.time() < end:
    d = s.read(256)
    if d: sys.stdout.buffer.write(d); sys.stdout.flush()
s.close()
```

### LVGL 9 / esp_lvgl_port v2
See working reference project under `/workspace/src/`

Key gotchas:
- `lvgl_port_display_cfg_t.io_handle` is required — pass the SPI panel IO handle, not NULL
- Use LVGL 9 API: `lv_display_t*`, `lv_button_create()`, `lv_obj_remove_flag()`, `lv_display_get_screen_active(disp)`
- `flags.swap_bytes=true`, `flags.buff_dma=true` for SPI displays

---

## 5. Write-Back Rule

At the end of any session where hardware behaviour was confirmed experimentally:
1. Update the relevant `knowledge/` file with the confirmed values
2. Trim any duplicate detail from `memory.md` — keep only the pointer and non-hardware gotchas
