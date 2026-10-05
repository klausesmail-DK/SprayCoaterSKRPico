@echo off
rem Launch the spray stage GUI, creating the virtualenv on first run.
rem
rem Windows twin of run.sh. The firmware tools are NOT duplicated here -
rem they live in spray_stage.board and are the same code on both platforms:
rem
rem     .venv\Scripts\python -m spray_stage.board push
rem     .venv\Scripts\python -m spray_stage.board check
rem     .venv\Scripts\python -m spray_stage.board reset
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo first run: creating .venv
    py -3 -m venv .venv
    if not exist ".venv\Scripts\python.exe" python -m venv .venv
    if not exist ".venv\Scripts\python.exe" (
        echo could not create a virtualenv.
        echo Install Python 3.11 or newer from python.org - the installer
        echo includes Tkinter, which the GUI needs - and tick "Add to PATH".
        exit /b 1
    )
    ".venv\Scripts\python.exe" -m pip install --quiet --upgrade pip
    ".venv\Scripts\python.exe" -m pip install --quiet -r requirements.txt
)

".venv\Scripts\python.exe" -m spray_stage.spray_gui %*
