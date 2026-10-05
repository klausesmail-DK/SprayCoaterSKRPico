#!/bin/bash
# Restart the board so main.py runs and serve() takes the serial line.
#   ./reset.sh                        finds the port itself
#   ./reset.sh /dev/cu.usbmodem101    explicit port
#
# A wrapper. The work lives in spray_stage/board.py, so Windows runs the same
# code instead of a parallel .bat file kept in step by hand; the command at
# the bench is unchanged.
set -euo pipefail
cd "$(dirname "$0")"
PY=.venv/bin/python
[ -x "$PY" ] || { echo "no $PY - run ./run.sh once to build the venv"; exit 1; }
exec "$PY" -m spray_stage.board reset "$@"
