<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# good, write me a short copy/pase summary of pins and board details, caveut - so I can use for claude code as starting point, consider also the pictures attached, knowledge on the internet.  from your feedback i assume I have the newer module - but please also explain how to watch out for th eolder one

**ESP32-Audio-Kit (Ai-Thinker v2.2, A1S Module Marking 2957 - ES8388 V2.3) Pin Summary \& Details**

**Board Overview**: ESP32 dual-core (240MHz, WiFi/BT, 4MB PSRAM), ES8388 codec (I2S/I2C audio, 3W/ch speakers), MicroSD (SPI), 2x mics, headphone, buttons (KEY1-6), JTAG/USB. Supports MP3 SD play via ESP-ADF/Arduino. Dimensions ~50x30mm dev board.[^1][^2][^3]

**Version Check (2957 = ES8388 Newer)**:

- **Newer ES8388 (V2.3, batches 2957/3478/A149/2762)**: Black-coated antenna, ES8388 under shield. Speakers: L (HPOUTL+/SPOLN-), R (HPOUTR+/SPORN-). Use ES8388 libs.[^4][^5][^6]
- **Older AC101 (V1.3, batches ~2974)**: Bare copper antenna, AC101 chip. Speakers: SPOLP/SPOLN, SPORP/SPORN. Init fails on ES8388 code—check serial "codec fail"; visual shield removal or test sine tone.[^7][^8]
- **Verify**: Run arduino-audiokit board=7 (ES8388); fail → AC101, swap board=5.[^2]

**Key Pins (for ES8388 2957 - Standard for MP3/SD Audio)**:

```
I2S Audio Out (Master): BCLK=27, WS/LRCK=25, DOUT=35 (stereo), MCLK=0 (ES8388 req)
I2C Codec Ctrl: SCL=32, SDA=33
SD Card (SPI3): CLK=14, CMD=15, D0=2, D1=4, D2=12, D3=13 (CS=13)
UART: TX=1/GPIO1, RX=3/GPIO3
Boot/Reset: BOOT=0/GPIO0 (w/MCLK), EN=EN
Exposed GPIOs: 0,2,4,5,12-15,18,19,21-23,34,36,39
Audio Outs (onboard JST): LOUT (HPOUTL+ Pin1 / SPOLN- Pin2), ROUT (HPOUTR+ / SPORN-)
Bias: HBIAS (mics), MBIAS
[file:1][web:5][web:12][web:37]
```

**Speaker Wiring**: Differential to 4Ω speakers on JST jacks—no external needed. Polarity: + to P (HPOUT*), - to N (SPOLN/SPORN).[^1]

**Caveats**:

- MCLK=0 essential for ES8388 stability (not AC101).
- SD mount before I2S init; conflicts on IO25/26/35.
- Volume via ES8388 reg (0-63); ALC off for clean MP3.
- Power: 5V USB, no >3.3V on GPIOs.
- Libs: Arduino `pschatzmann/arduino-audiokit` (board 7), ESP-ADF pipeline_sdcard_mp3 (ES8388 config).[^9][^2]
- Pics (overview.pdf): Confirms pin tables, speaker labels, module placement (check antenna).[^1]

Copy-paste into Claude: Start with `#define AUDIOKIT_BOARD 7` + MP3Decoder("/sd/song.mp3"). Test sine first![^2]

<div align="center">⁂</div>

[^1]: overview.pdf

[^2]: https://github.com/pschatzmann/arduino-audiokit

[^3]: https://vdoc.ai-thinker.com/en/esp32-audio-kit

[^4]: https://github.com/Ai-Thinker-Open/ESP32-A1S-AudioKit/issues/26

[^5]: https://www.pschatzmann.ch/home/2021/12/06/the-ai-thinker-audio-kit-experience-or-nothing-is-right/

[^6]: https://github.com/Ai-Thinker-Open/ESP32-A1S-AudioKit/issues/42

[^7]: https://www.youtube.com/watch?v=8UB3fYPjqSk

[^8]: https://www.fambach.net/preview-esp32-audio-kit-esp32/

[^9]: https://github.com/Ai-Thinker-Open/ESP32-A1S-AudioKit

