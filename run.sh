#!/bin/bash
# Launch the spray stage GUI, creating the virtualenv on first run.
set -e
cd "$(dirname "$0")"
if [ ! -d .venv ]; then
    echo "first run: creating .venv"
    python3 -m venv .venv
    .venv/bin/pip install --quiet --upgrade pip
    .venv/bin/pip install --quiet -r requirements.txt
fi
exec .venv/bin/python -m spray_stage.spray_gui "$@"
