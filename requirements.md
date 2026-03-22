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
Plays all `.mp3` files found on an SD card in a loop. WS2812B 30-LED spectrum analyzer driven by FFT of the PCM stream. KEY3/KEY4 navigate previous/next track.

**Requirements:**
- Read all `.mp3` files from SD card root, loop forever
- Clean audio output via ES8388 (same codec init as beep)
- WS2812B LED spectrum analyzer, 30 LEDs, GPIO22
- KEY3 (GPIO19) = previous track, KEY4 (GPIO23) = next track

**Implementation notes:**
- SD must mount before I2S init — GPIO25/26 conflict during init sequence
- SDMMC slot 1, 1-bit mode: CLK=GPIO14, CMD=GPIO15, D0=GPIO2
- `chmorgan/esp-audio-player` (1.1.0) + `chmorgan/esp-libhelix-mp3` — handles MP3 decode, sample-rate switching via `clk_set_fn`, `force_stereo=true` for mono files
- `clk_set_fn` signature: `(uint32_t rate, uint32_t bits, i2s_slot_mode_t ch)` — NOT `uint32_t ch`
- Library calls `fclose()` on the FILE* when done — caller must not close it
- `espressif/esp-dsp` (1.7.1) for 512-point FFT: `dsps_fft2r_init_fc32`, `dsps_fft2r_fc32`, `dsps_bit_rev2r_fc32`, `dsps_wind_hann_f32`
- PCM tap in `i2s_write_cb`: stereo int16 downmixed to mono float, pushed to `xStreamBuffer`
- LED task on core 0 (priority 3): reads FFT_SIZE floats, applies Hann window, FFT, log-compress, asymmetric smooth, beat detection, HSV render
- Audio decode task pinned to core 1 (priority 5)
- KEY1/GPIO36 unusable — input-only, no internal pull-up; KEY2/GPIO13 unusable — SD conflict
- KEY3 (GPIO19) and KEY4 (GPIO23) confirmed working: active LOW, internal pull-up, 40 ms debounce
- `audio_player_stop()` triggers IDLE callback → semaphore → main loop picks up `s_next_track_override`
- Button task on core 0 (priority 2), polls at 20 ms, 400 ms lockout after press
