#!/bin/bash
# Build "Spray Stage.app" - a standalone macOS bundle, so the GUI starts
# from the Dock without a terminal or a virtualenv.
#
#   ./packaging/build_macos.sh
#
# The result lands in dist/. Both build/ and dist/ are gitignored.
#
# macOS only: PyInstaller cannot cross-compile, so a Windows .exe has to be
# built on Windows.
set -euo pipefail
cd "$(dirname "$0")/.."

VENV=.venv
[ -d "$VENV" ] || { echo "no $VENV - run ./run.sh once to create it"; exit 1; }

"$VENV"/bin/pip install -q pyinstaller

rm -rf build dist
"$VENV"/bin/pyinstaller \
    --noconfirm --clean --windowed \
    --name "Spray Stage" \
    --osx-bundle-identifier com.linehq.spraystage \
    --paths . \
    --hidden-import serial.tools.list_ports \
    --collect-submodules serial \
    packaging/launcher.py

echo
echo "--- self-test (imports from inside the bundle, not from the venv) ---"
"dist/Spray Stage.app/Contents/MacOS/Spray Stage" --selftest

echo
echo "Built: dist/Spray Stage.app"
echo
echo "The bundle is unsigned, so the FIRST launch needs right-click > Open"
echo "and then Open again in the dialog. After that it starts normally."
echo
echo "To put it on the Desktop:"
echo "  cp -R 'dist/Spray Stage.app' ~/Desktop/"
echo
echo "NOTE: the firmware is NOT in the bundle. ./push.sh still needs the"
echo "project folder and the venv."
