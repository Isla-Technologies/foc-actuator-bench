#!/bin/bash
# ODrive Actuator Bench - start on macOS.
#
# Run it from Terminal:   bash start_mac.command
# (or double-click it in Finder; if macOS blocks it, use the Terminal command above).
#
# The first run creates a private Python environment in ".venv" and installs what the app needs
# (needs internet, takes a few minutes). After that it starts right away.

cd "$(dirname "$0")" || exit 1

python_ok() {
    "$1" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' >/dev/null 2>&1
}

find_python() {
    local candidate path
    for candidate in \
        python3.13 python3.12 python3.11 python3.10 python3.14 \
        /Library/Frameworks/Python.framework/Versions/3.13/bin/python3 \
        /Library/Frameworks/Python.framework/Versions/3.12/bin/python3 \
        /Library/Frameworks/Python.framework/Versions/3.11/bin/python3 \
        /Library/Frameworks/Python.framework/Versions/3.10/bin/python3 \
        /Library/Frameworks/Python.framework/Versions/3.14/bin/python3 \
        /opt/homebrew/bin/python3 /usr/local/bin/python3 python3; do
        path=$(command -v "$candidate" 2>/dev/null) || continue
        # /usr/bin/python3 is Apple's 3.9 stub; running it can pop up an Xcode install dialog.
        [ "$path" = "/usr/bin/python3" ] && continue
        if python_ok "$path"; then
            echo "$path"
            return 0
        fi
    done
    return 1
}

if [ ! -x .venv/bin/python ]; then
    PY=$(find_python)
    if [ -z "$PY" ]; then
        echo ""
        echo "Python 3.10 or newer is needed and was not found."
        echo "  1. Download the macOS installer (Python 3.12 or 3.13) from https://www.python.org/downloads/"
        echo "  2. Run it, then run this script again."
        open "https://www.python.org/downloads/macos/" 2>/dev/null
        exit 1
    fi
    echo "Using $("$PY" --version 2>&1) from $PY"
    echo "Creating the app environment in .venv ..."
    "$PY" -m venv .venv || { echo "Could not create the environment."; exit 1; }
fi

if ! cmp -s requirements.txt .venv/installed-requirements.txt; then
    echo "Installing the required packages (first run only, a few minutes) ..."
    .venv/bin/python -m pip install --upgrade pip || { echo "Install failed: check the internet connection and run this again."; exit 1; }
    .venv/bin/python -m pip install -r requirements.txt || { echo "Install failed: check the internet connection and run this again."; exit 1; }
    .venv/bin/python -m pip install odrive \
        || echo "Note: the optional official 'odrive' library was not installed. The USB serial connection still works."
    cp requirements.txt .venv/installed-requirements.txt
fi

exec .venv/bin/python -m actuator_bench "$@"
