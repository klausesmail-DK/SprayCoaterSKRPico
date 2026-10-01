"""PyInstaller entry point.

PyInstaller freezes a script, not a `-m module`, so this is the thinnest
wrapper around spray_stage.spray_gui.main that will do.

`--selftest` checks that the frozen bundle actually carries its
dependencies. PyInstaller compresses pure-Python modules into an archive
inside the executable, so you cannot confirm they are there by looking at
the .app - the only honest check is importing them from inside the bundle,
which is what the build script does after it builds.
"""

import sys

if "--selftest" in sys.argv:
    import tkinter

    from serial.tools import list_ports

    from spray_stage.spray_gui import KINDS, SprayApp  # noqa: F401
    from spray_stage.stage import Envelope

    env = Envelope()
    ports = [p.device for p in list_ports.comports()]
    print(f"pyserial    OK  - ports: {ports or '(none attached)'}")
    print(f"tkinter     OK  - Tk {tkinter.TkVersion}")
    print(f"spray_gui   OK  - types: {list(KINDS.values())}")
    print(f"envelope    OK  - X {env.lo_x:g}..{env.hi_x:g}  "
          f"Y {env.lo_y:g}..{env.hi_y:g}  feed <= {env.max_feed_xy:g}")
    raise SystemExit(0)

from spray_stage.spray_gui import main  # noqa: E402

raise SystemExit(main())
