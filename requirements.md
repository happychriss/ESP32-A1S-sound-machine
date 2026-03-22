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

### 2. SD card MP3 player

**Status:** Complete (step 1) / Not started (step 2 — LED)
**Source:** `/workspace/src/player/`

**Description:**
Plays all `.mp3` files found on an SD card in a loop. Step 2 will add a WS2812B LED equalizer (30 LEDs, GPIO22) driven by FFT analysis of the PCM stream.

**Requirements:**
- Read all `.mp3` files from SD card root, loop forever
- Clean audio output via ES8388 (same codec init as beep)
- Step 2: WS2812B LED bar visualizer, equalizer / disco effect

**Implementation notes:**
- SD must mount before I2S init — GPIO25/26 conflict during init sequence
- SDMMC slot 1, 1-bit mode: CLK=GPIO14, CMD=GPIO15, D0=GPIO2
- `chmorgan/esp-audio-player` (1.1.0) + `chmorgan/esp-libhelix-mp3` — handles MP3 decode, sample-rate switching via `clk_set_fn`, `force_stereo=true` for mono files
- `clk_set_fn` signature: `(uint32_t rate, uint32_t bits, i2s_slot_mode_t ch)` — NOT `uint32_t ch`
- Library calls `fclose()` on the FILE* when done — caller must not close it
- PCM write callback (`i2s_write_cb`) is the tap point for step 2 FFT
- `espressif/led_strip` (3.0.3) already in `idf_component.yml`, LED_GPIO=GPIO22 defined, stub TODO in `i2s_write_cb`
- Audio decode task pinned to core 1
