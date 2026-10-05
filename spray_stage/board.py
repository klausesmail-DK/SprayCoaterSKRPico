"""Push the firmware to the board, inspect it, or restart it - on any platform.

    python -m spray_stage.board push      firmware onto the board, then reset
    python -m spray_stage.board check     what is on it, and does it import
    python -m spray_stage.board reset     restart it so serve() owns the line

Add a port to skip the search:

    python -m spray_stage.board push COM5

WHY THIS IS PYTHON AND NOT A SHELL SCRIPT

The scripts this replaces found the board by globbing /dev/cu.usbmodem*,
which exists on macOS and nowhere else. Everything they did afterwards -
mpremote, two file copies, a reset - is already portable, and the port is
already discoverable from find_ports(). Writing the job once here means
Windows runs the same commands rather than keeping a parallel pair of .bat
files in step with them.

mpremote is invoked as `python -m mpremote` rather than by its script name,
because on Windows the console script lands in .venv\\Scripts as an .exe and
on macOS in .venv/bin - and the interpreter running this module already knows
where its own environment is.

THE ONE THING TO REMEMBER
mpremote sends Ctrl-C when it attaches, which stops serve(). Nothing starts
it again by itself, so `check` ends by telling you to `reset`, and `push`
resets for you. A GUI that opens the port and then gets no reply is almost
always a board left sitting at the REPL by an earlier mpremote command.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from .pico_controller import find_ports

HERE = Path(__file__).resolve().parent.parent
FIRMWARE = HERE / "pico_stage" / "skr_pico_stage.py"
BOOT = HERE / "pico_stage" / "main.py"


def resolve_port(argv: list[str]) -> str:
    """The port to talk to: whatever was asked for, or the best candidate.

    Refuses to choose between several rather than picking one, because a wrong
    guess here writes firmware to somebody else's microcontroller.
    """
    if argv:
        return argv[0]
    ports = find_ports()
    if not ports:
        raise SystemExit("no serial ports found - is the board plugged in?")
    if len(ports) > 1:
        lines = "\n".join(f"    {p.device}  {p.description}" for p in ports)
        raise SystemExit(
            f"several ports to choose from - name the one you want:\n{lines}")
    return ports[0].device


def mpr(port: str, *args: str, check: bool = True) -> int:
    """One mpremote command. Echoed first, so a failure is reproducible."""
    cmd = [sys.executable, "-m", "mpremote", "connect", port, *args]
    print("$ " + " ".join(cmd[3:]))
    try:
        proc = subprocess.run(cmd, check=False)
    except FileNotFoundError:
        raise SystemExit(
            "mpremote is not installed in this environment - "
            f"{sys.executable} -m pip install mpremote") from None
    if check and proc.returncode != 0:
        raise SystemExit(f"mpremote failed ({proc.returncode})")
    return proc.returncode


def push(port: str) -> int:
    for src in (FIRMWARE, BOOT):
        if not src.exists():
            raise SystemExit(f"{src} not found")
        mpr(port, "fs", "cp", str(src), f":{src.name}")
    mpr(port, "fs", "ls")
    mpr(port, "reset")
    print("\nreset - wait ~5 s (3 s Ctrl-C window, then the drivers "
          "configure), then start the GUI")
    return 0


def check(port: str) -> int:
    # Against the local file rather than a number in a comment: a hard-coded
    # size goes stale the first time the firmware is edited, and "is what I
    # just pushed actually on the board" is the whole question here.
    print(f"local  {FIRMWARE.name}  {FIRMWARE.stat().st_size} bytes")
    print("--- files on the board ---")
    mpr(port, "fs", "ls")
    print("\n--- does the firmware import? (a traceback here is why "
          "main.py dies) ---")
    mpr(port, "exec", "import skr_pico_stage as s; print(s.BANNER)", check=False)
    print("\nthat attach stopped serve() - run `reset` before using the GUI")
    return 0


def reset(port: str) -> int:
    mpr(port, "reset")
    print("wait ~5 s (3 s Ctrl-C window, then the drivers configure), "
          "then start the GUI")
    return 0


ACTIONS = {"push": push, "check": check, "reset": reset}


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] not in ACTIONS:
        raise SystemExit(f"usage: python -m spray_stage.board "
                         f"{{{'|'.join(ACTIONS)}}} [port]")
    action = argv.pop(0)
    port = resolve_port(argv)
    print(f"port {port}")
    return ACTIONS[action](port)


if __name__ == "__main__":
    raise SystemExit(main())
