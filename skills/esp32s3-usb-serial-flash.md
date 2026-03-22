---
name: esp32s3-usb-serial-flash
description: Flash and monitor ESP32-S3 over USB-CDC (ttyACM/ttyESP) — handles write timeout, usb_reset bootloader entry, and non-TTY serial reading
---

# ESP32-S3 USB-CDC Serial Flash & Monitor

## Context

The ESP32-S3 has a built-in **USB-Serial/JTAG** peripheral that exposes a USB-CDC device (`/dev/ttyACM*`, often aliased to `/dev/ttyESP`).

This interface behaves differently from a standard UART-to-USB bridge:

- Reads always work (firmware output is buffered by the chip)
- **Writes block/timeout** when the running firmware is not consuming USB-CDC input — this causes `idf.py flash` to fail with `Write timeout`
- `--before=default_reset` (DTR/RTS toggling) does **not** reliably reset an ESP32-S3 into bootloader mode over USB-CDC
- `idf_monitor` and `miniterm` require an interactive TTY and will fail in non-TTY environments (Claude Code, CI, etc.)

---

## Flashing

### Why `idf.py flash` fails

`idf.py flash` uses `--before=default_reset` (DTR/RTS) which does not work over USB-CDC when the running firmware blocks writes. The result is:

```
A serial exception error occurred: Write timeout
```

### Fix: use `esptool` directly with `--before=usb_reset`

`usb_reset` triggers the ESP32-S3's USB CDC reset sequence, putting it into ROM bootloader mode before writing.

```bash
. /opt/esp/esp-idf/export.sh

python -m esptool --chip esp32s3 -p /dev/ttyESP -b 460800 \
  --before=usb_reset --after=hard_reset write_flash \
  --flash_mode dio --flash_freq 80m --flash_size detect \
  0x0     build/bootloader/bootloader.bin \
  0x8000  build/partition_table/partition-table.bin \
  0x10000 build/<app_name>.bin
```

### Flash addresses (standard ESP-IDF layout)

| Binary | Address |
|--------|---------|
| bootloader.bin | `0x0` |
| partition-table.bin | `0x8000` |
| app.bin | `0x10000` |

### Alternative: 1200-baud touch (manual trigger)

If `usb_reset` is unavailable or unreliable, open the port at 1200 baud briefly — this signals the CDC device to reboot into bootloader mode:

```python
import serial, time
s = serial.Serial('/dev/ttyESP', 1200, timeout=1, xonxoff=False, rtscts=False, dsrdtr=False)
s.dtr = False
time.sleep(0.25)
s.close()
time.sleep(3)  # wait for re-enumeration
```

### Manual fallback

Hold **BOOT** button, press **RESET**, release BOOT — then flash with `--before=no_reset`.

---

## Serial Monitor (non-TTY environment)

`idf.py monitor` and `miniterm` require a real TTY. In Claude Code or CI, use a raw Python reader:

```python
import serial, time, sys

s = serial.Serial('/dev/ttyESP', 115200, timeout=1,
                  xonxoff=False, rtscts=False, dsrdtr=False)
end = time.time() + 15  # read for N seconds
while time.time() < end:
    chunk = s.read(256)
    if chunk:
        sys.stdout.buffer.write(chunk)
        sys.stdout.flush()
s.close()
```

---

## Verify chip before flashing

```bash
python -m esptool --chip esp32s3 -p /dev/ttyESP -b 460800 \
  --before=usb_reset --no-stub flash_id
```

Expected output includes `USB mode: USB-Serial/JTAG` and detected flash size.

---

## Summary of reset modes

| `--before` value | When to use |
|-----------------|-------------|
| `usb_reset` | **Default for ESP32-S3 USB-CDC** — triggers USB CDC bootloader entry |
| `default_reset` | UART bridges with DTR/RTS wired (not USB-CDC) |
| `no_reset` | Device already in bootloader mode (held BOOT+RESET manually) |
| `no_reset_no_sync` | Debugging only |
