#!/bin/bash
# FOC Actuator Bench - start on Linux:   bash start_linux.sh
# Serial access needs your user in the "dialout" group:  sudo usermod -aG dialout $USER  (then log in again)

cd "$(dirname "$0")" || exit 1

if [ ! -x .venv/bin/python ]; then
    PY=""
    for candidate in python3.13 python3.12 python3.11 python3.10 python3.14 python3; do
        if command -v "$candidate" >/dev/null 2>&1 \
            && "$candidate" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' 2>/dev/null; then
            PY=$(command -v "$candidate")
            break
        fi
    done
    if [ -z "$PY" ]; then
        echo "Python 3.10 or newer is needed (e.g. sudo apt install python3 python3-venv)."
        exit 1
    fi
    "$PY" -m venv .venv || { echo "Could not create the environment (install python3-venv?)."; exit 1; }
fi

if ! cmp -s requirements.txt .venv/installed-requirements.txt; then
    echo "Installing the required packages (first run only) ..."
    .venv/bin/python -m pip install --upgrade pip && .venv/bin/python -m pip install -r requirements.txt \
        || { echo "Install failed: check the internet connection and run this again."; exit 1; }
    .venv/bin/python -m pip install odrive || echo "Note: the optional official 'odrive' library was not installed."
    cp requirements.txt .venv/installed-requirements.txt
fi

exec .venv/bin/python -m actuator_bench "$@"
