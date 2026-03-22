# CLAUDE.md

## For Claude — Session Bootstrap

At the start of every session:
1. Read `/workspace/requirements.md` — project type, what is being built, current status. It links to the `skills/<type>-setup.md` that defines the working conventions for this project type.
2. Read all `.md` files in `/workspace/skills/` — active skills and conventions, including the setup file named in `requirements.md`
3. Read all `.md` files in `/workspace/knowledge/` — confirmed component and technology config

## For Claude — File Ownership

- `README.md` — for humans: how to set up the container, clone the template, configure serial device. Do not use it as a reference for your own work.
- `CLAUDE.md` — for you: bootstrap and runtime context. This file.
- `requirements.md` — for you: what is being built. Start here for every task.

## For Claude — Runtime Environment

You are running **inside** the dev container. This means:
- Your workspace is `/workspace` — all project files are here
- You have direct access to the filesystem, shell, ESP-IDF toolchain, and `/dev/ttyESP`
- Docker commands (`docker-compose`, `docker exec`) are host-side tools — do not run them
- The sections below (Container Setup, Project Isolation, etc.) are instructions for the **human operator** managing the container from the host — they are not relevant to your work

---

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a **VS Code Dev Container template** for running Claude Code in an isolated Docker container. It is designed to be copied and customized per project. The current instance name is set via `COMPOSE_PROJECT_NAME` in `.env`.

Tip: copy `.env.example` → `.env` when you fork/copy the template.

## Container Setup

The development environment runs entirely inside Docker. There is no local build/test pipeline outside the container.

**Start the container:**
```bash
docker-compose up -d
```

**Rebuild after Dockerfile changes:**
```bash
docker-compose up -d --build
```

**Open a shell inside the running container:**
```bash
docker exec -it ${COMPOSE_PROJECT_NAME}-dev zsh
```

The VS Code Dev Containers extension attaches to the `dev` service defined in `docker-compose.yml` using the config in `.devcontainer/devcontainer.json`.

## Project Isolation

`COMPOSE_PROJECT_NAME` in `.env` controls the container name (`${COMPOSE_PROJECT_NAME}-dev`) and scopes all named Docker volumes. When copying this template for a new project, update this value to avoid collisions with other copies.

Named volumes:
- `claude-home` → `/home/ubuntu/.claude` (Claude config, isolated per project)
- `devcontainer-commandhistory` → `/commandhistory` (shell history)

## Claude Config Seeding

`claude-config/` is a seed template for `/home/ubuntu/.claude`. On first container start (when the volume is empty), `entrypoint.sh` copies `claude-config/` into the volume. After that, changes persist in the volume and `claude-config/` is ignored.

To ship default Claude memory/skills/MCP files with this template, place them under `claude-config/` using the same relative paths you want under `~/.claude/`.

## Serial Device Access

The container maps a host serial device to `/dev/ttyESP`. The default is `/dev/ttyACM0`; override with `SERIAL_DEVICE=/dev/ttyUSB0` in `.env`. The `entrypoint.sh` automatically resolves group permissions so the non-root `ubuntu` user can access the device.

## Optional Firewall

To restrict outbound network access to an allowlist (GitHub, npm, Anthropic API, VS Code), run inside the container:
```bash
sudo /usr/local/bin/init-firewall.sh
```

This is not enabled by default.

## Key Files

| File | Purpose |
|------|---------|
| `.env` | Sets `COMPOSE_PROJECT_NAME` and optional `SERIAL_DEVICE` |
| `docker-compose.yml` | Defines the `dev` service, volumes, and device mapping |
| `.devcontainer/Dockerfile` | Full devcontainer image (Ubuntu, Node.js, Python, Claude Code CLI, zsh) |
| `.devcontainer/devcontainer.json` | VS Code devcontainer config pointing at `docker-compose.yml` |
| `.devcontainer/entrypoint.sh` | Startup: fixes workspace ownership, serial permissions, seeds Claude config |
| `.devcontainer/init-firewall.sh` | Optional outbound network allowlist |
| `claude-config/` | Seed template for `/home/ubuntu/.claude` (first-start only) |
