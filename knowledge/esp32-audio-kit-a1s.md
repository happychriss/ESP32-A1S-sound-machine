# ESP32-Audio-Kit (Ai-Thinker ESP32-A1S V2.3, ES8388)

Source documents: `external-docs/003625_ESP32-A1S-datasheet.pdf`, `external-docs/overview.pdf`, `external-docs/good, write me a short copy_pase summary of pins a.md`

---

## Board Overview

| Field | Value |
|-------|-------|
| Board | Ai-Thinker ESP32-Audio-Kit |
| Module | ESP32-A1S V2.2 (batch marking A541) — confirmed on hand |
| Chip | ESP32 dual-core LX6, 240 MHz (NOT ESP32-S3) |
| PSRAM | 64 Mb (8 MB) integrated |
| Flash | External SPI, 4 MB typical |
| Audio codec | ES8388 (I2S + I2C control) |
| Dimensions | Module: 31.5 × 19.0 × 3.1 mm |
| Supply | 3.0–3.6 V, >500 mA (USB 5V via onboard reg) |

**ESP-IDF target: `esp32` (not `esp32s3`)**

---

## Version Identification — ES8388 vs AC101

| Feature | V2.3 (ES8388 — this board) | V1.3 (AC101 — older) |
|---------|---------------------------|----------------------|
| Batch markings | 2957, 3478, A149, 2762, A541 | ~2974 |
| Antenna coating | Black-coated | Bare copper |
| Shield | ES8388 under RF shield | AC101 visible |
| arduino-audiokit | `AUDIOKIT_BOARD 7` | `AUDIOKIT_BOARD 5` |
| Speaker pins | HPOUTL+/SPOLN−, HPOUTR+/SPORN− | SPOLP/SPOLN, SPORP/SPORN |

**Verification**: Run init — serial "codec fail" means wrong chip variant.

---

## Serial / Flash Interface

- USB-UART bridge (CP2102 or CH340) → `/dev/ttyESP` (device node `188,0` = ttyUSB class)
- Standard DTR/RTS reset — **normal `idf.py flash` works** (unlike ESP32-S3 USB-CDC)
- Baud: 115200 default, 460800 for flashing

```bash
source /opt/esp/esp-idf/export.sh
idf.py -p /dev/ttyESP flash
# or directly with esptool:
python -m esptool --chip esp32 -p /dev/ttyESP -b 460800 \
  --before=default_reset --after=hard_reset write_flash \
  --flash_mode dio --flash_freq 80m --flash_size detect \
  0x1000  build/bootloader/bootloader.bin \
  0x8000  build/partition_table/partition-table.bin \
  0x10000 build/<app>.bin
```

Note: ESP32 bootloader is at **0x1000** (not 0x0 like ESP32-S3).

---

## Pin Table — ESP32-A1S Module (38-pin SMD)

| Pin# | Name | GPIO / Function |
|------|------|-----------------|
| 1 | GND | Ground |
| 2 | 3V3 | 3.3 V power |
| 3 | SENSOR_VN | GPI39, ADC1_CH3 |
| 4 | SENSOR_VP | GPI36, ADC1_CH0 |
| 5 | IO34 | GPI34, ADC1_CH6 |
| 6 | IO0 | GPIO0 — MCLK for ES8388, also BOOT button |
| 7 | IO14 | GPIO14 — SD_CLK |
| 8 | IO12 | GPIO12 — SD_DATA2 |
| 9 | IO13 | GPIO13 — SD_DATA3 / CS |
| 10 | IO15 | GPIO15 — SD_CMD |
| 11 | IO2 | GPIO2 — SD_DATA0 |
| 12 | IO4 | GPIO4 — SD_DATA1 |
| 13 | HBIAS | Mic bias (internal 1 kΩ to AVCC) |
| 14 | MIC2N | ES8388 RIN2 |
| 15 | MIC1N | ES8388 RIN1 |
| 16 | MBIAS | Mic bias |
| 17 | MIC1P | ES8388 LIN1 |
| 18 | MIC2P | ES8388 LIN2 |
| 19–20 | GND | Ground |
| 21 | LINEINR | ES8388 RIN2 (shared with MIC2 — cannot use simultaneously) |
| 22 | LINEINL | ES8388 LIN2 |
| 23 | NC | Not connected |
| 24 | SPORN | ES8388 ROUT1 — Right speaker **negative** |
| 25 | NC | Not connected |
| 26 | SPOLN | ES8388 LOUT1 — Left speaker **negative** |
| 27 | HPOUTL | ES8388 LOUT2 — Left speaker/headphone **positive** |
| 28 | HPOUTR | ES8388 ROUT2 — Right speaker/headphone **positive** |
| 29 | IO5 | GPIO5 |
| 30 | IO18 | GPIO18 |
| 31 | IO23 | GPIO23 |
| 32 | IO19 | GPIO19 |
| 33 | IO22 | GPIO22 |
| 34 | IO21 | GPIO21 |
| 35 | EN | Chip enable (active high) |
| 36 | TXD0 | GPIO1, UART0 TX |
| 37 | RXD0 | GPIO3, UART0 RX |
| 38 | GND | Ground |

---

## I2S Audio — ES8388 (Master)

| Signal | GPIO | Notes |
|--------|------|-------|
| MCLK | GPIO0 | Required for ES8388 stability — shared with BOOT button |
| BCLK | GPIO27 | Bit clock |
| WS/LRCK | GPIO25 | Left/right word select |
| DOUT (codec → ESP) | GPIO35 | ADC data from ES8388 |
| DIN (ESP → codec) | GPIO26 | DAC data to ES8388 (confirmed working) |

---

## I2C Codec Control — ES8388

| Signal | GPIO |
|--------|------|
| SCL | GPIO32 |
| SDA | GPIO33 |

ES8388 I2C address: `0x10` (7-bit)

---

## SD Card — SPI/SDIO (SPI3 / HSPI)

| Signal | GPIO |
|--------|------|
| CLK | GPIO14 |
| CMD/MOSI | GPIO15 |
| D0/MISO | GPIO2 |
| D1 | GPIO4 |
| D2 | GPIO12 |
| D3/CS | GPIO13 |

Mount SD before I2S init — GPIO25/26 conflict risk during init sequence.

---

## Speaker Output — V2.3 Wiring (ES8388)

Differential output to 4 Ω speakers on JST jacks:

| Channel | Positive | Negative |
|---------|----------|----------|
| Left | HPOUTL (pin 27) | SPOLN (pin 26) |
| Right | HPOUTR (pin 28) | SPORN (pin 24) |

---

## Speaker Amplifier Enable — NS4150

| Pin | GPIO | Polarity |
|-----|------|----------|
| PA_ENABLE | GPIO21 | **HIGH = on**, LOW = shutdown |

Must be driven HIGH after ES8388 init for any sound from the speaker JST jacks. The headphone jack (HPOUTL/HPOUTR) bypasses the NS4150 and does not need this.

```c
gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT);
gpio_set_level(GPIO_NUM_21, 1);
```

---

## Buttons — KEY1–KEY6 (# unconfirmed — from arduino-audiokit community)

| Button | GPIO |
|--------|------|
| KEY1 | GPIO36 |
| KEY2 | GPIO13 (conflicts with SD CS) |
| KEY3 | GPIO19 |
| KEY4 | GPIO23 |
| KEY5 | GPIO18 |
| KEY6 | GPIO5 |

---

## Audio Format Support

MP3, AAC, FLAC, WAV, OGG, OPUS, AMR, TS. Sources: HTTP, HLS, SPIFFS, SD card, A2DP, HFP.

---

## Key Caveats

- **MCLK = GPIO0** is essential for ES8388 — shared with BOOT; do not hold it LOW at startup
- **MIC2 and LINEIN are shared** on ES8388 — cannot use simultaneously
- **Volume**: ES8388 register (0–63); disable ALC for clean MP3 playback
- **GPIO0** doubles as MCLK and BOOT — hold HIGH during normal operation
- **SD before I2S**: mount SD card before I2S initialisation to avoid GPIO25/26 conflicts
- **3.3 V I/O only** — do not connect >3.3 V to any GPIO

---

## ES8388 — Confirmed Working Approach (ESP-IDF v5.5.3)

**Do not hand-roll ES8388 register writes.** Use `espressif/esp_codec_dev` from the IDF
component registry — it handles the full init sequence, CHIPPOWER state machine, mixer
routing, PA enable, and volume control correctly.

### Component setup

`main/idf_component.yml`:
```yaml
dependencies:
  idf:
    version: '>=4.1.0'
  espressif/esp_codec_dev: '*'
```

`sdkconfig.defaults` (add):
```
CONFIG_CODEC_ES8388_SUPPORT=y
```

`main/CMakeLists.txt` REQUIRES: add `espressif__esp_codec_dev`

### I2C address

`ES8388_CODEC_DEFAULT_ADDR = 0x20` — the driver uses the 8-bit shifted form of the
7-bit address 0x10. Pass `bus_handle` from `i2c_new_master_bus()`.

### Initialisation order (critical)

1. `i2s_new_channel()` + `i2s_channel_init_std_mode()` + `i2s_channel_enable()` — **MCLK must be running first**
2. `vTaskDelay(100 ms)` — let MCLK stabilise before codec power-up
3. `audio_codec_new_i2s_data()` → `audio_codec_new_i2c_ctrl()` → `audio_codec_new_gpio()` → `es8388_codec_new()` → `esp_codec_dev_new()`
4. `esp_codec_dev_open()` — applies sample rate, bit depth, channels to codec
5. `esp_codec_dev_set_out_vol()` — 0–100 scale

### I2S clock source

Use `I2S_CLK_SRC_DEFAULT` (PLL_F160M). **Do NOT use `I2S_CLK_SRC_APLL`** — APLL
has significant phase jitter on ESP32 that produces loud broadband noise at the DAC output.
The sample rate will be ~44117 Hz instead of exactly 44100 Hz, which is inaudible.

### es8388_codec_cfg_t for DAC-only playback

```c
es8388_codec_cfg_t cfg = {
    .ctrl_if     = ctrl_if,          // from audio_codec_new_i2c_ctrl()
    .gpio_if     = gpio_if,          // from audio_codec_new_gpio()
    .codec_mode  = ESP_CODEC_DEV_WORK_MODE_DAC,
    .master_mode = false,            // ESP32 is I2S master, ES8388 is slave
    .pa_pin      = GPIO_NUM_21,      // NS4150 amp enable — driver sets HIGH
    .pa_reverted = false,
};
```

### Speaker frequency

The onboard JST speaker jacks drive 4 Ω speakers via the NS4150 class-D amp. Small
speakers (3 W, 4 Ω mini) have Fs ≈ 200–400 Hz — tones below ~300 Hz will not be
audible. Use ≥ 1 kHz for test tones.

Working reference: `/workspace/src/beep/` (full ESP-IDF project)

---

## Recommended Libraries

| Use case | Library |
|----------|---------|
| ESP-IDF (recommended) | `espressif/esp_codec_dev` from IDF component registry |
| Arduino | `pschatzmann/arduino-audiokit` (`AUDIOKIT_BOARD 7`) |
| ESP-IDF / ESP-ADF pipeline | `pipeline_sdcard_mp3_example` with ES8388 board config |
