#!/bin/bash
set -e
cd "$(dirname "$0")/.."
# EIM activation requires sourcing inside bash -c.
exec /bin/bash -c '
    if [ -z "$IDF_PATH" ]; then
        activation="$HOME/.espressif/tools/activate_idf_v6.1.sh"
        if [ ! -f "$activation" ]; then
            echo "Activate ESP-IDF 6.1 first (source path/to/esp-idf/export.sh)." >&2
            exit 1
        fi
        source "$activation" >/dev/null || exit 1
    fi
    python_bin="${IDF_PYTHON_ENV_PATH:+$IDF_PYTHON_ENV_PATH/bin/}python"
    exec "$python_bin" tools/flash_guard.py "$@"
' bash "$@"
