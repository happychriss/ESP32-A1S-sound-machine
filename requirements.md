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

### 2. SD card MP3 player with LED spectrum + keys

**Status:** Complete
**Source:** `/workspace/src/player/`

**Description:**
Plays all `.mp3` files found on an SD card in a loop. WS2812B 30-LED mood show driven by FFT of the PCM stream. KEY1 pause/resume, KEY3/KEY4 prev/next, KEY5/KEY6 volume.

**Requirements:**
- Read all `.mp3` files from SD card root, loop forever
- Clean audio output via ES8388 (same codec init as beep)
- WS2812B LED mood show, 30 LEDs, GPIO22
- KEY1 (GPIO36) = pause/resume, KEY3 (GPIO19) = previous, KEY4 (GPIO23) = next
- KEY5 (GPIO18) = volume down, KEY6 (GPIO5) = volume up

**Implementation notes:**
- SD must mount before I2S init — GPIO25/26 conflict during init sequence
- SDMMC slot 1, 1-bit mode: CLK=GPIO14, CMD=GPIO15, D0=GPIO2
- `chmorgan/esp-audio-player` (1.1.0) + `chmorgan/esp-libhelix-mp3` — handles MP3 decode, sample-rate switching via `clk_set_fn`, `force_stereo=true` for mono files
- `clk_set_fn` signature: `(uint32_t rate, uint32_t bits, i2s_slot_mode_t ch)` — NOT `uint32_t ch`
- Library calls `fclose()` on the FILE* when done — caller must not close it
- `espressif/esp-dsp` (1.7.1) for 512-point FFT: `dsps_fft2r_init_fc32`, `dsps_fft2r_fc32`, `dsps_bit_rev2r_fc32`, `dsps_wind_hann_f32`
- PCM tap in `i2s_write_cb`: stereo int16 downmixed to mono float, pushed to `xStreamBuffer`
- LED task on core 0 (priority 3): 4-layer composited show — ambient palette rotation + beat pulse + bass blob + sparkles
- 8 log bands (60 Hz–20 kHz), per-band AGC (τ~10 s) × global RMS gate
- Beat detection: spectral flux > flux_mean + 1.2 × flux_std (adaptive stddev threshold)
- Section change: cosine novelty on 8-band vectors → palette cycle (5 palettes, min 5 s gap)
- Disco snap: hue_offset jumps 40–100° on every beat → all LED colors shift to new positions
- Audio decode task pinned to core 1 (priority 5)
- KEY1/GPIO36 — input-only pin, no internal pull-up; board external pull-up makes it usable
- KEY2/GPIO13 unusable — SD D3 conflict
- KEY3–KEY6: active LOW, internal pull-up, 40 ms debounce, 400 ms lockout
- Pause: `gpio_set_level(PA_ENABLE_PIN, 0)` → `audio_player_stop()` → main loop blocks on `s_resume_sem`
- Resume: button clears `s_is_paused` → gives `s_resume_sem` → main loop replays same track
- Prev/Next while paused: set override + give `s_resume_sem` (no second stop — already stopped)
- `player_event_cb` PLAYING event: `gpio_set_level(PA_ENABLE_PIN, 1)` unmutes amp on track start
- Button task on core 0 (priority 2), polls at 20 ms, 400 ms lockout after press
