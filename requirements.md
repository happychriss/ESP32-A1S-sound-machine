# Project Requirements

## Project Type

- Embedded firmware — follow `/workspace/skills/embedded-project-setup.md` for working conventions, folder structure, knowledge flow, and build workflow.

---

## Hardware

**Board:** Ai-Thinker ESP32-Audio-Kit (ESP32-A1S V2.2, batch A541, ES8388 codec)
**Full config:** `/workspace/knowledge/esp32-audio-kit-a1s.md`

---

## Functional Specification

### 1. Beep (sine tone test)

**Status:** Complete
**Source:** `/workspace/src/beep/`

**Description:**
Plays a 1 kHz sine tone on the left speaker JST jack in a continuous loop. Used to validate the full I2S → ES8388 → NS4150 → speaker chain.

**Requirements:**
- Clean audio output on left speaker JST jack
- No white noise

**Implementation notes:**
- Use `espressif/esp_codec_dev` — never hand-roll ES8388 register writes
- `I2S_CLK_SRC_DEFAULT` only — APLL causes broadband white noise on ESP32
- I2S start → 100 ms delay → codec init (MCLK must be stable first)
- Volume 60/100 via `esp_codec_dev_set_out_vol()`
- Test tones must be ≥ 1 kHz (small speaker Fs ≈ 200–400 Hz)

---

### 2. SD card MP3 player + Bluetooth A2DP sink with LED spectrum + keys

**Status:** Complete
**Source:** `/workspace/src/player/`

**Description:**
Plays all `.mp3` files found on an SD card in a loop. Long-press KEY1 (2 s) switches to Bluetooth A2DP sink mode — phone streams audio through the same codec and LED show. Short-press KEY1 pauses/resumes SD playback. KEY3/KEY4 prev/next, KEY5/KEY6 volume. WS2812B 30-LED mood show runs in both modes.

**Requirements:**
- SD card: read all `.mp3` files from root, loop forever
- Clean audio output via ES8388
- Bluetooth A2DP sink: long-press KEY1 enters 30 s pairing window (LEDs blink blue), phone connects via SSP "Just Works", audio streams through codec with LED show
- WS2812B LED mood show, 30 LEDs, GPIO22
- KEY1 (GPIO36) short = pause/resume, long (2 s) = BT mode toggle
- KEY3 (GPIO19) = previous, KEY4 (GPIO23) = next, KEY5 (GPIO18) = vol down, KEY6 (GPIO5) = vol up

**Implementation notes — SD/MP3:**
- SD must mount before I2S init — GPIO25/26 conflict during init sequence
- SDMMC slot 1, 1-bit mode: CLK=GPIO14, CMD=GPIO15, D0=GPIO2
- `chmorgan/esp-audio-player` (1.1.0) + `chmorgan/esp-libhelix-mp3` — handles MP3 decode, `force_stereo=true` for mono files
- `clk_set_fn` signature: `(uint32_t rate, uint32_t bits, i2s_slot_mode_t ch)` — NOT `uint32_t ch`
- Library calls `fclose()` on the FILE* when done — do not close it
- Pause: `gpio_set_level(PA_ENABLE_PIN, 0)` → `audio_player_stop()` → main loop blocks on `s_resume_sem`
- Resume: button gives `s_resume_sem` → main loop replays same track → PLAYING → PA unmuted
- Never use `audio_player_pause()` — crashes I2S DMA on this board

**Implementation notes — Bluetooth:**
- `CONFIG_BT_BLE_ENABLED=n` required — without it `btm_ble_init` asserts on boot even with `BTDM_CTRL_MODE_BR_EDR_ONLY=y`
- Do NOT call `esp_bt_controller_mem_release(ESP_BT_MODE_BLE)` when BLE is compile-disabled
- Custom 3 MB partition (`partitions.csv`) + `CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y` — BT stack pushes binary to ~1.1 MB
- A2DP data callback must be non-blocking: push to `StreamBufferHandle_t`; `bt_audio_task` on Core 1 drains to codec — blocking in callback drops BT link
- SBC sample rate CIE byte 0 is **one-hot bitmask**: bit5=44.1k, bit4=48k, bit6=32k, bit7=16k. Index formula `rates[(byte>>6)&3]` is wrong (maps 44.1k→16k)
- `esp_codec_dev_open()` re-asserts `.pa_pin` HIGH — must `gpio_set_level(PA_ENABLE_PIN, 0)` immediately after every close+open in CFG callback
- PA unmuted on first audio frame in `bt_audio_task` (not on STARTED event) — eliminates blurp noise gap between connect and first play
- Heap budget: stream buffers 8 KB each, led_task stack 3 KB (FFT arrays are `static`), bt_audio_task 2 KB — leaves ~23 KB free at pairing time

**Implementation notes — LED show:**
- `espressif/esp-dsp` (1.7.1), 512-point FFT with Hann window
- PCM tap in both `i2s_write_cb` (SD) and `a2dp_data_cb` (BT): stereo int16 → mono float → `xStreamBuffer`
- LED task core 0 (priority 3): 4-layer composited show — ambient palette rotation + beat pulse + bass blob + sparkles
- 8 log bands (60 Hz–20 kHz), per-band AGC (τ~10 s) × global RMS gate
- Beat detection: spectral flux > flux_mean + 1.2×flux_std
- Section change: cosine novelty → palette cycle (5 palettes, min 5 s gap)
- Disco snap: hue_offset jumps 40–100° on every beat

**Implementation notes — buttons:**
- KEY1/GPIO36: input-only pin, no internal pull-up; board external pull-up. Long-press = 100 polls × 20 ms
- KEY2/GPIO13 unusable — SD D3 conflict
- KEY3–KEY6: active LOW, internal pull-up, 40 ms debounce, 400 ms lockout
- Button task on core 0 (priority 2), 20 ms poll
