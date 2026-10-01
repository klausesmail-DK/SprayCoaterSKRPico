"""Dry-run a recipe: print the G-code, the bounds check and the time estimate.

Nothing is sent and nothing moves. This is the check worth doing after
migrating a recipe and before putting paint through it - it shows the exact
lines the stage will receive, including the lead distances a gradient needs,
and it refuses the same things the GUI would refuse.

    python -m spray_stage.check_recipe PicoPatterns/*.json
    python -m spray_stage.check_recipe --quiet PicoPatterns/*.json

The planner still lives inside spray_gui, which imports Tkinter at module
scope. On a machine without it - a headless box, a CI runner - a stub stands
in, because none of the planning touches a widget.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

try:  # pragma: no cover - only bites on headless installs
    import tkinter  # noqa: F401
except ImportError:
    import types
    for name in ("tkinter", "tkinter.ttk", "tkinter.filedialog",
                 "tkinter.messagebox", "tkinter.simpledialog"):
        mod = types.ModuleType(name)
        sys.modules[name] = mod
    sys.modules["tkinter"].Tk = type("Tk", (), {})
    sys.modules["tkinter"].StringVar = type("StringVar", (), {})
    sys.modules["tkinter"].Event = type("Event", (), {})
    sys.modules["tkinter"].Menu = type("Menu", (), {})
    for sub in ("ttk", "filedialog", "messagebox", "simpledialog"):
        setattr(sys.modules["tkinter"], sub, sys.modules[f"tkinter.{sub}"])

from .spray_gui import (  # noqa: E402
    SprayLine,
    estimate_seconds,
    format_duration,
    plan_moves,
)
from .stage import Envelope  # noqa: E402


def load(path: Path) -> list[SprayLine]:
    data = json.loads(path.read_text())
    out = []
    for raw in data.get("lines", []):
        out.append(SprayLine(**{k: v for k, v in raw.items()
                                if k in SprayLine.__dataclass_fields__}))
    return out, float(data.get("travel_feed", 1800) or 1800)


def check(path: Path, env: Envelope, quiet: bool) -> int:
    lines, travel = load(path)
    print(f"\n=== {path.name}  ({len(lines)} entries, travel F{travel:g}) ===")
    bad = 0
    for i, ln in enumerate(lines, 1):
        for e in ln.validate(env):
            print(f"  ! line {i}: {e}")
            bad += 1
    plan = plan_moves(lines, travel, env)
    gcode = [p for k, p in plan if k == "gcode"]
    if not quiet:
        for kind, payload in plan:
            mark = {"gcode": "   ", "pause": "  ~", "dwell": "  =", "note": " # "}[kind]
            print(f"{mark}{payload}")
    secs = estimate_seconds(plan)
    print(f"  {len(gcode)} moves, about {format_duration(secs)}"
          f"{'  -- ' + str(bad) + ' BOUNDS ERROR(S)' if bad else ''}")
    return bad


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("files", nargs="+", type=Path)
    ap.add_argument("--quiet", action="store_true", help="totals only")
    args = ap.parse_args(argv)
    env = Envelope()
    print(f"envelope  X {env.lo_x:g}..{env.hi_x:g}  Y {env.lo_y:g}..{env.hi_y:g}"
          f"  feed <= {env.max_feed_xy:g}")
    bad = sum(check(p, env, args.quiet) for p in args.files)
    print(f"\n{bad} bounds error(s) in {len(args.files)} file(s)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
