# knowledge/

Curated reference notes — one file per component, library, or technology. This folder is the single source of truth for confirmed configuration, integration details, and validated quirks.

Each file is derived from raw source material in `../external-docs/` and updated whenever experimental work confirms or corrects a value. Unconfirmed values are marked clearly.

Typical file contents:
- Pin/signal tables (hardware components)
- Interface configuration (SPI, I2C, UART — clock speed, polarity, addressing)
- Initialisation sequences and driver/library settings
- Confirmed quirks and gotchas

**Write-back rule:** when behaviour is confirmed or corrected through testing, update the relevant file here. Do not store component-level facts in `memory.md` — keep only a short pointer there.
