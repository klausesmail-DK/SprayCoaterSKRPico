#!/bin/bash
# Copy the firmware to the Pico and restart it.
#   ./push.sh                        finds the port itself
#   ./push.sh /dev/cu.usbmodem101    explicit port
set -e
cd "$(dirname "$0")"
[ -d .venv ] || { echo "no .venv - run ./run.sh once first"; exit 1; }
MPR=.venv/bin/mpremote
[ -x "$MPR" ] || { .venv/bin/pip install --quiet mpremote; }
PORT="${1:-$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)}"
# Interactive zsh does not strip trailing comments, so a pasted
# "./push.sh   # do the thing" arrives here as PORT='#'. Anything that is
# not a device path is not a port.
case "$PORT" in
    /dev/*) ;;
    "")  echo "no /dev/cu.usbmodem* found - is the board plugged in?"; exit 1 ;;
    *)   echo "'$PORT' is not a serial port - pass /dev/cu.usbmodem101 or nothing"; exit 1 ;;
esac
echo "port $PORT"
$MPR connect "$PORT" fs cp pico_stage/skr_pico_stage.py :skr_pico_stage.py
$MPR connect "$PORT" fs cp pico_stage/main.py :main.py
$MPR connect "$PORT" fs ls
$MPR connect "$PORT" reset
echo "reset - wait ~5 s, then ./run.sh"
