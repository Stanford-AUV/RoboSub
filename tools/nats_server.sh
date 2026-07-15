#!/usr/bin/env bash
# Run the NATS message broker on the sub for manual (joystick/keyboard) teleop.
#
# The host-side scripts (joystick_local.py / keyboard_local.py) on the pilot
# laptop publish controller state to this broker over the tether, and the ROS
# `manual/joystick` (or `manual/keyboard`) node subscribes to it here on the sub.
#
# Binds 0.0.0.0:4222 so the laptop can reach it over the tether ethernet.
# The ROS node connects locally (nats://localhost:4222); the laptop connects to
# nats://<this-sub-ip>:4222 (see joystick_local.py --host / NATS_URL).
#
# The nats-server binary is git-ignored (14 MB). If it's missing this script
# downloads the matching arch build once into tools/bin/.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="$HERE/bin/nats-server"
PORT="${NATS_PORT:-4222}"
NATS_VERSION="${NATS_VERSION:-v2.10.22}"

if [ ! -x "$BIN" ]; then
    case "$(uname -m)" in
        aarch64|arm64) ARCH=arm64 ;;
        x86_64|amd64)  ARCH=amd64 ;;
        *) echo "Unsupported arch $(uname -m); install nats-server manually to $BIN" >&2; exit 1 ;;
    esac
    NAME="nats-server-${NATS_VERSION}-linux-${ARCH}"
    URL="https://github.com/nats-io/nats-server/releases/download/${NATS_VERSION}/${NAME}.tar.gz"
    echo "nats-server not found; downloading $URL ..."
    mkdir -p "$HERE/bin"
    TMP="$(mktemp -d)"
    curl -fsSL "$URL" -o "$TMP/nats.tgz"
    tar xzf "$TMP/nats.tgz" -C "$TMP"
    mv "$TMP/$NAME/nats-server" "$BIN"
    chmod +x "$BIN"
    rm -rf "$TMP"
fi

exec "$BIN" -a 0.0.0.0 -p "$PORT" "$@"
