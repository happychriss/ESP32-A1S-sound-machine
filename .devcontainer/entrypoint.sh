#!/usr/bin/env bash
set -euo pipefail

# When the workspace is mounted with root ownership (common with certain
# Docker/volume setups), VS Code attaching as the non-root user can't create
# files at the repo root. Fix it once on startup, then drop privileges.

if [ "$(id -u)" = "0" ]; then
  # Some tooling expects the non-root user to at least be able to traverse
  # /root (readability is still governed by file permissions).
  chmod 0755 /root 2>/dev/null || true

  if [ -d /workspace ]; then
    desired_uid="$(id -u ubuntu)"
    desired_gid="$(id -g ubuntu)"
    current_uid="$(stat -c '%u' /workspace 2>/dev/null || echo 0)"
    current_gid="$(stat -c '%g' /workspace 2>/dev/null || echo 0)"

    if [ "$current_uid" != "$desired_uid" ] || [ "$current_gid" != "$desired_gid" ]; then
      chown ubuntu:ubuntu /workspace || true
      # Ensure the workspace stays writable even if files are created by different
      # processes: setgid keeps the group consistent.
      chmod 2775 /workspace || true
    fi
  fi

  # If a serial device is passed through, ensure the non-root user can access it.
  # When binding /dev/tty* from the host, the device's GID is also from the host.
  # Create/resolve a matching group inside the container and add ubuntu to it.
  for dev in /dev/ttyESP /dev/ttyACM0 /dev/ttyUSB0; do
    if [ -e "$dev" ]; then
      dev_gid="$(stat -c '%g' "$dev" 2>/dev/null || true)"
      if [ -n "$dev_gid" ]; then
        dev_group="$(getent group "$dev_gid" | cut -d: -f1 || true)"
        if [ -z "$dev_group" ]; then
          dev_group="serial"
          groupadd -g "$dev_gid" "$dev_group" 2>/dev/null || true
        fi
        usermod -aG "$dev_group" ubuntu 2>/dev/null || true
      fi
      break
    fi
  done

  # Seed Claude config from the repo template, but only if the target config dir is empty.
  # Claude's config directory (/home/ubuntu/.claude) is persisted (via a volume/mount),
  # and this provides a project-specific baseline template on first start.
  if [ -d /workspace/claude-config ]; then
    mkdir -p /home/ubuntu/.claude
    chown -R ubuntu:ubuntu /home/ubuntu/.claude || true
    if [ -z "$(ls -A /home/ubuntu/.claude 2>/dev/null || true)" ]; then
      cp -a /workspace/claude-config/. /home/ubuntu/.claude/ 2>/dev/null || true
      chown -R ubuntu:ubuntu /home/ubuntu/.claude || true
    fi
  fi

  # Ensure the user's local bin directory is on PATH.
  # Claude Code's native install lives at /home/ubuntu/.local/bin/claude, and we want it
  # to take precedence over any system/npm-global installation.
  mkdir -p /home/ubuntu/.local/bin
  chown -R ubuntu:ubuntu /home/ubuntu/.local || true
  export PATH="/home/ubuntu/.local/bin:${PATH}"
  if [ ! -f /etc/profile.d/00-ubuntu-local-bin.sh ]; then
    cat > /etc/profile.d/00-ubuntu-local-bin.sh <<'EOF'
export PATH="/home/ubuntu/.local/bin:${PATH}"
EOF
    chmod 0644 /etc/profile.d/00-ubuntu-local-bin.sh || true
  fi

  # Re-exec as the non-root user while preserving argv.
  # NOTE: `su -c` does not reliably preserve argv boundaries; it also drops the
  # first argument when used with `bash -c`, which breaks commands like
  # `sleep infinity` (you end up trying to exec just `infinity`).
  exec sudo -E -H -u ubuntu -- "$@"
fi

# If no command is provided, keep the container running
if [ $# -eq 0 ]; then
  exec sleep infinity
fi

exec "$@"
