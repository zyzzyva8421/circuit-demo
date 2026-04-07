#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$ROOT_DIR/build"
APP="$BUILD_DIR/circuit-demo"

if [[ ! -x "$APP" ]]; then
    echo "Executable not found: $APP" >&2
    echo "Build the project first." >&2
    exit 1
fi

NODE_BIN=""
if command -v node >/dev/null 2>&1; then
    NODE_BIN="$(command -v node)"
elif [[ -x "$HOME/.nvm/versions/node/v22.22.0/bin/node" ]]; then
    NODE_BIN="$HOME/.nvm/versions/node/v22.22.0/bin/node"
fi

if [[ -z "$NODE_BIN" ]]; then
    echo "Node.js not found. Install Node.js or add it to PATH." >&2
    exit 1
fi

NODE_DIR="$(dirname "$NODE_BIN")"
CLEAN_PATH="$NODE_DIR:/usr/local/bin:/usr/bin:/bin"

cd "$BUILD_DIR"

# Pass through all arguments to the application
exec env -i \
    HOME="$HOME" \
    USER="${USER:-$(id -un)}" \
    LOGNAME="${LOGNAME:-${USER:-$(id -un)}}" \
    SHELL="${SHELL:-/bin/bash}" \
    TERM="${TERM:-xterm-256color}" \
    LANG="${LANG:-C.UTF-8}" \
    LC_ALL="${LC_ALL:-${LANG:-C.UTF-8}}" \
    DISPLAY="${DISPLAY:-}" \
    WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
    XAUTHORITY="${XAUTHORITY:-}" \
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}" \
    QT_IM_MODULE="${QT_IM_MODULE:-}" \
    PATH="$CLEAN_PATH" \
    "$APP" "$@"