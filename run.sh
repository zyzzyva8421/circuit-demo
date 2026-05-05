#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$ROOT_DIR/build"
APP="$BUILD_DIR/circuit-demo"

RUN_TIMEOUT_SEC="${RUN_TIMEOUT_SEC:-180}"
RUN_KILL_AFTER_SEC="${RUN_KILL_AFTER_SEC:-10}"
RUN_MAX_VMEM_MB="${RUN_MAX_VMEM_MB:-8192}"
RUN_MAX_CPU_SEC="${RUN_MAX_CPU_SEC:-0}"

require_non_negative_int() {
    local name="$1"
    local value="$2"
    if [[ ! "$value" =~ ^[0-9]+$ ]]; then
        echo "$name must be a non-negative integer, got: $value" >&2
        exit 1
    fi
}

require_non_negative_int "RUN_TIMEOUT_SEC" "$RUN_TIMEOUT_SEC"
require_non_negative_int "RUN_KILL_AFTER_SEC" "$RUN_KILL_AFTER_SEC"
require_non_negative_int "RUN_MAX_VMEM_MB" "$RUN_MAX_VMEM_MB"
require_non_negative_int "RUN_MAX_CPU_SEC" "$RUN_MAX_CPU_SEC"

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

ulimit -c 0

if (( RUN_MAX_VMEM_MB > 0 )); then
    ulimit -Sv $((RUN_MAX_VMEM_MB * 1024))
fi

if (( RUN_MAX_CPU_SEC > 0 )); then
    ulimit -St "$RUN_MAX_CPU_SEC"
fi

TIMEOUT_CMD=()
if (( RUN_TIMEOUT_SEC > 0 )); then
    if ! command -v timeout >/dev/null 2>&1; then
        echo "timeout command not found, but RUN_TIMEOUT_SEC=$RUN_TIMEOUT_SEC was requested" >&2
        exit 1
    fi
    TIMEOUT_CMD=(timeout --foreground --signal=TERM --kill-after="${RUN_KILL_AFTER_SEC}s" "${RUN_TIMEOUT_SEC}s")
fi

echo "[run.sh] timeout=${RUN_TIMEOUT_SEC}s kill_after=${RUN_KILL_AFTER_SEC}s max_vmem=${RUN_MAX_VMEM_MB}MB max_cpu=${RUN_MAX_CPU_SEC}s" >&2

# Pass through all arguments to the application
exec "${TIMEOUT_CMD[@]}" env -i \
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