FROM ubuntu:25.04

ARG TZ
ENV TZ="$TZ"

ARG CLAUDE_CODE_VERSION=latest
ARG DEBIAN_FRONTEND=noninteractive

# Install basic development tools + network tooling (mirrors upstream Claude Code devcontainer)
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    bash \
    ca-certificates \
    coreutils \
    curl \
    wget \
    less \
    git \
    procps \
    sudo \
    fzf \
    zsh \
    man-db \
    unzip \
    gnupg2 \
    openssh-client \
    gh \
    iptables \
    ipset \
    iproute2 \
    dnsutils \
    aggregate \
    jq \
    nano \
    vim \
    build-essential \
    python3 \
    python3-pip \
    python3-venv \
    python3-serial \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Install Node.js (LTS) from NodeSource for predictable versions
ARG NODE_MAJOR=20
RUN set -eux; \
  curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --dearmor -o /usr/share/keyrings/nodesource.gpg; \
  echo "deb [signed-by=/usr/share/keyrings/nodesource.gpg] https://deb.nodesource.com/node_${NODE_MAJOR}.x nodistro main" > /etc/apt/sources.list.d/nodesource.list; \
  apt-get update; \
  apt-get install -y --no-install-recommends nodejs; \
  apt-get clean; \
  rm -rf /var/lib/apt/lists/*

# Create a predictable non-root user for VS Code.
RUN set -eux; \
  getent group dialout >/dev/null || groupadd dialout; \
  getent group ubuntu >/dev/null || groupadd ubuntu; \
  id -u ubuntu >/dev/null 2>&1 || useradd -m -s /bin/bash -g ubuntu ubuntu; \
  usermod -aG dialout ubuntu; \
  echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu; \
  chmod 0440 /etc/sudoers.d/ubuntu

# Persist shell history (directory is mounted/created by devcontainer runtime if desired).
RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=/commandhistory/.bash_history" \
  && mkdir -p /commandhistory \
  && touch /commandhistory/.bash_history \
  && chown -R ubuntu:ubuntu /commandhistory

# Help with orientation
ENV DEVCONTAINER=true

# Ensure ubuntu user can install global npm packages without root
RUN mkdir -p /usr/local/share/npm-global \
  && chown -R ubuntu:ubuntu /usr/local/share/npm-global

# Create workspace and Claude config directories
RUN mkdir -p /workspace /home/ubuntu/.claude \
  && chown -R ubuntu:ubuntu /workspace /home/ubuntu/.claude

WORKDIR /workspace

ARG GIT_DELTA_VERSION=0.18.2
RUN ARCH=$(dpkg --print-architecture) && \
  wget "https://github.com/dandavison/delta/releases/download/${GIT_DELTA_VERSION}/git-delta_${GIT_DELTA_VERSION}_${ARCH}.deb" && \
  dpkg -i "git-delta_${GIT_DELTA_VERSION}_${ARCH}.deb" || (apt-get update && apt-get -f install -y && apt-get clean && rm -rf /var/lib/apt/lists/*) && \
  rm "git-delta_${GIT_DELTA_VERSION}_${ARCH}.deb"

# Install global packages as non-root user
USER ubuntu
ENV NPM_CONFIG_PREFIX=/usr/local/share/npm-global
ENV PATH=$PATH:/usr/local/share/npm-global/bin

# Prefer zsh as default shell + editor defaults
ENV SHELL=/bin/zsh
ENV EDITOR=nano
ENV VISUAL=nano

ARG ZSH_IN_DOCKER_VERSION=1.2.0
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v${ZSH_IN_DOCKER_VERSION}/zsh-in-docker.sh)" -- \
  -p git \
  -p fzf \
  -a "[[ -r /usr/share/doc/fzf/examples/key-bindings.zsh ]] && source /usr/share/doc/fzf/examples/key-bindings.zsh" \
  -a "[[ -r /usr/share/doc/fzf/examples/completion.zsh ]] && source /usr/share/doc/fzf/examples/completion.zsh" \
  -a "export PROMPT_COMMAND='history -a' && export HISTFILE=/commandhistory/.bash_history" \
  -x

RUN npm install -g @anthropic-ai/claude-code@${CLAUDE_CODE_VERSION}

# Firewall helper script (optional; useful for MCP + marketplace update allowlists)
COPY .devcontainer/init-firewall.sh /usr/local/bin/init-firewall.sh
USER root
RUN chmod +x /usr/local/bin/init-firewall.sh \
  && echo "ubuntu ALL=(root) NOPASSWD: /usr/local/bin/init-firewall.sh" > /etc/sudoers.d/ubuntu-firewall \
  && chmod 0440 /etc/sudoers.d/ubuntu-firewall

COPY .devcontainer/entrypoint.sh /usr/local/share/devcontainer/entrypoint.sh
RUN chmod +x /usr/local/share/devcontainer/entrypoint.sh

CMD ["/usr/local/share/devcontainer/entrypoint.sh", "sleep", "infinity"]
