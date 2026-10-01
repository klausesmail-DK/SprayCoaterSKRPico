#!/bin/bash
# Restart the board so main.py runs and serve() takes the serial line again.
#
# Needed after any mpremote command other than `reset`: mpremote sends Ctrl-C
# when it attaches, which stops serve(), and nothing starts it again by
# itself. The symptom is a GUI that opens the port and then gets no reply.
#
#   ./reset.sh                        finds the port itself
#   ./reset.sh /dev/cu.usbmodem101    explicit port
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
echo "resetting $PORT"
"$MPR" connect "$PORT" reset
echo "wait ~5 s (3 s Ctrl-C window, then the drivers configure), then ./run.sh"
