"""Runs on the Pico at boot. Hands the USB serial line to the host program.

THE ESCAPE WINDOW, AND WHY IT EXISTS

serve() reads stdin. So does Thonny's raw-paste protocol, which is how it
loads files, lists the filesystem and evaluates anything you type. Once
serve() is running they fight over the same bytes, and Thonny fails with
"Unexpected read during raw paste" or a SyntaxError from a half-swallowed
command. Worse, Thonny's usual remedy - Stop/Restart - soft-reboots the
board, which runs this file again, which starts serve() again.

Hence the pause below. For three seconds after boot nothing has claimed the
serial line, so a Ctrl-C lands cleanly and drops you at the REPL with the
board otherwise untouched. Thonny's Stop/Restart sends exactly that.

The drivers are configured inside serve()'s import, AFTER the window, so
the escape happens before anything touches the motors.
"""

import time

DELAY_S = 3

print("main.py: starting serve() in %d s - Ctrl-C now for the REPL" % DELAY_S)
try:
    time.sleep(DELAY_S)
except KeyboardInterrupt:
    print("stopped before serve() - the REPL is yours, nothing configured")
    raise SystemExit

import skr_pico_stage as stage

stage.serve()
