#!/bin/bash
# What is actually on the board, and does the firmware import cleanly?
# Run this when the GUI connects but gets no reply.
#
# NOTE: this stops serve() - mpremote sends Ctrl-C to attach. Run ./reset.sh
# afterwards before going back to the GUI.
set -euo pipefail
cd "$(dirname "$0")"
MPR=.venv/bin/mpremote
[ -x "$MPR" ] || { echo "no $MPR - run ./run.sh once to build the venv"; exit 1; }
PORT="${1:-$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)}"
# Interactive zsh does not strip trailing comments, so a pasted
# "./push.sh   # do the thing" arrives here as PORT='#'. Anything that is
# not a device path is not a port.
case "$PORT" in
    /dev/*) ;;
    "")  echo "no /dev/cu.usbmodem* found - is the board plugged in?"; exit 1 ;;
    *)   echo "'$PORT' is not a serial port - pass /dev/cu.usbmodem101 or nothing"; exit 1 ;;
esac
echo "--- files on the board (skr_pico_stage.py should be ~51700 bytes) ---"
"$MPR" connect "$PORT" fs ls
echo
echo "--- does the firmware import? (a traceback here is why main.py dies) ---"
"$MPR" connect "$PORT" exec "import skr_pico_stage as s; print(s.BANNER)"
echo
echo "now run ./reset.sh"
