FROM ubuntu:25.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    ca-certificates curl git \
    nodejs npm \
    python3 python3-serial \
  && npm install -g @anthropic-ai/claude-code \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
CMD ["/bin/bash"]

