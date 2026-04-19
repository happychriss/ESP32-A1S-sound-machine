# Dietmars-Soundbox — ESP32 Audio Player

SD card MP3 player + Bluetooth A2DP sink with WS2812B LED mood show, running on an Ai-Thinker ESP32-Audio-Kit (ESP32-A1S V2.2, ES8388 codec).

**Features:**
- Plays all `.mp3` files from SD card root in alphabetical filename order; `_welcome.mp3` always plays first
- KEY1 short-press = pause/resume; long-press (2 s) = toggle Bluetooth A2DP sink mode
- KEY3/KEY4 = prev/next track; KEY5/KEY6 = volume down/up
- BT device name: **Dietmars-Soundbox** — phone streams audio via SBC with the same LED show
- 30-LED WS2812B strip on GPIO22: 5-layer composited show (ambient + beat burst + bass blob + sparkles + picture frame), 8-band FFT with per-band AGC and beat detection

**Source:** `src/player/` — see `requirements.md` for full spec and implementation notes.

---

# Claude Code devcontainer template

This repo is structured to work out-of-the-box with VS Code **Dev Containers** using the custom root [docker-compose.yml](docker-compose.yml) service `dev`.

## Quick setup

1. Copy `.env.example` → `.env` and set `COMPOSE_PROJECT_NAME` (recommended for uniqueness).
2. (Optional) If your serial device is not `/dev/ttyACM0`, set `SERIAL_DEVICE=/dev/ttyUSB0` in `.env`.
	- If you have no serial device, comment out the `devices:` section in [docker-compose.yml](docker-compose.yml).
3. Open this folder in VS Code → run **Dev Containers: Reopen in Container**.

Alternative (no VS Code): run `docker compose up -d` from the repo root.


## Repo structure

| Path | Audience | Purpose |
|------|----------|---------|
| `README.md` | Human | You are here — setup and usage guide |
| `CLAUDE.md` | Claude | Session bootstrap: tells Claude to read `requirements.md`, all `skills/`, and all `knowledge/` at the start of every session |
| `requirements.md` | Human + Claude | Project type declaration (links to the matching `skills/<type>-setup.md`) + functional specification |
| `skills/` | Claude | Working conventions loaded each session. One file per concern; the `<type>-setup.md` file named in `requirements.md` defines the project-type-specific conventions |
| `knowledge/` | Claude | Confirmed config and integration notes — one file per component or technology, derived from `external-docs/` and updated as things are validated |
| `external-docs/` | Claude (read-only) | Raw reference material: datasheets, schematics, vendor docs |
| `src/` | Both | Source projects |
| `.devcontainer/` | Human | VS Code devcontainer config and Dockerfile |
| `claude-config/` | Human | Seed template for Claude's config volume (first-start only) |

## What’s included

- **Devcontainer image**: [.devcontainer/Dockerfile](.devcontainer/Dockerfile)
	- Ubuntu base image, non-root `ubuntu` user
	- Node.js (LTS) + `@anthropic-ai/claude-code`
	- Common CLI tooling (git, gh, jq, fzf, zsh, iptables/ipset, etc.)
	- Optional firewall helper script (allowlist-based)
- **Devcontainer config**: [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json)
	- Uses root compose file: `../docker-compose.yml`
	- Persists shell history in a named volume
- **Project Claude template config**: [claude-config/](claude-config/)
	- On first container start, if `/home/ubuntu/.claude` is empty, it is seeded from `/workspace/claude-config`.

## Serial device access

The compose service maps a host serial device into the container as `/dev/ttyESP`.

- Default: host `/dev/ttyACM0` → container `/dev/ttyESP`
- Override with `.env`: set `SERIAL_DEVICE=/dev/ttyUSB0`
- Or override per command: `SERIAL_DEVICE=/dev/ttyUSB0 docker compose up -d`
- Or edit the `devices:` mapping directly in [docker-compose.yml](docker-compose.yml)

If you don’t have a serial device on the host (or the default path doesn’t exist), Docker Compose will fail to start the container. In that case, either set `SERIAL_DEVICE` to a real device path, or comment out the `devices:` section in [docker-compose.yml](docker-compose.yml).

Inside the container, the startup entrypoint detects the device GID and adds `ubuntu` to the owning group so non-root access works reliably.

## Customizing the container / project name

The compose file uses `COMPOSE_PROJECT_NAME` (from a local `.env` file) to build the dev container name.

Setup when you copy this template:

1. Copy `.env.example` → `.env`
2. Edit `.env` and set `COMPOSE_PROJECT_NAME` to something unique per copy (e.g. `my-new-project`)

Do not commit `.env` (it can grow secrets over time).

Notes:

- `docker compose` automatically reads `.env` when you run it from the same directory as the compose file.
- If you run compose from somewhere else, use `docker compose --env-file /path/to/.env ...`.

Result:

- The dev container name becomes `${COMPOSE_PROJECT_NAME}-dev`
- The compose “project name” also becomes `COMPOSE_PROJECT_NAME`, so networks/volumes won’t collide across copies

## Claude config persistence

Each copied template gets an **isolated** Claude config directory inside the container at `/home/ubuntu/.claude`.

This is persisted via a **Docker named volume** (`claude-home`) which is automatically scoped by `COMPOSE_PROJECT_NAME`, so different projects do not share Claude memory/settings.

The repo’s [claude-config/](claude-config/) is used as a **seed template** (copied only when the target dir is empty).

Where to put Claude “memory / skills / MCP” files:

- Claude reads/writes under `/home/ubuntu/.claude` inside the container.
- To ship defaults with this template, put them into [claude-config/](claude-config/) using the same paths you want under `~/.claude/`.
- After the first start, changes persist in the project’s `claude-home` volume (so they stay isolated per copied template).

## Persistence — what survives a container rebuild

| Location | Volume | Survives rebuild? |
|----------|--------|-------------------|
| `/home/ubuntu/.claude` | `claude-home` | ✅ Yes |
| `/home/ubuntu/.ssh` | `ssh-home` | ✅ Yes |
| `/workspace` | host bind-mount | ✅ Yes (it's the repo) |
| Everything else in `/home/ubuntu` | none | ❌ No |

The Claude API credentials (`.claude/.credentials.json`) are in the persisted volume and survive rebuilds. If Claude asks you to re-authenticate, the token has expired — not a storage issue.

## GitHub access — deploy key setup

To allow Claude to push/pull from a single GitHub repo without touching your other repositories:

**Inside the container:**
```bash
ssh-keygen -t ed25519 -C "my-project deploy key" -f ~/.ssh/my-project -N ""
cat ~/.ssh/my-project.pub   # copy this
```

Add to `~/.ssh/config`:
```
Host github-my-project
    HostName github.com
    User git
    IdentityFile ~/.ssh/my-project
    IdentitiesOnly yes
```

**On GitHub** (browser): repo → Settings → Deploy keys → Add deploy key → paste public key → tick "Allow write access"

**Test and set remote:**
```bash
ssh -T github-my-project    # should greet you with the repo name
git remote add origin git@github-my-project:yourname/my-project.git
git push -u origin main
```

The key is scoped to one repo only — it cannot access any of your other GitHub repositories.

## Optional: network allowlist (firewall)

If you want to restrict outbound access to typical development endpoints (GitHub, npm registry, VS Code marketplace/update URLs, Anthropic API, etc.), run:

`sudo /usr/local/bin/init-firewall.sh`

This is optional and is not enabled automatically.

