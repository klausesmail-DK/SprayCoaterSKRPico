"""Convert Shapeoko spray recipes to the Pico stage's coordinate system.

Both machines measure negative from a back-right home, so the frame is the
same; what differs is how much of it there is. The Shapeoko ran to -845 in X
and -850 in Y, the stage to -94 and -139 - about a twentieth of the area.

So this is a translation, not a conversion: the pattern is moved so its
bounding box sits centred in the stage's travel, lead zones included. Line
spacing, lengths, pass directions and the gradient itself are untouched,
because that is the part that carries the experiment.

Feeds above the stage's ceiling are handled by scaling the WHOLE recipe down
by one factor rather than clipping the fast end. Clipping would flatten the
top of a gradient and silently change the coating; scaling keeps the ratio
between the two feeds, which is what a calibration strip actually depends
on. The factor is reported so you know it happened.

Usage:
    python -m spray_stage.migrate_recipes SprayPatterns/*.json
    python -m spray_stage.migrate_recipes --out-dir PicoPatterns *.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .stage import DEFAULT_ACCEL, DEFAULT_TRAVEL_FEED, JUNCTION_MMS, Envelope


def _lead(feed: float, env: Envelope, axis: str) -> float:
    return env.lead(feed, axis) if feed > 0 else 0.0


def _extent(lines: list[dict], env: Envelope) -> tuple[float, float, float, float]:
    """Bounding box of the flipped pattern, lead zones included."""
    xs: list[float] = []
    ys: list[float] = []
    for ln in lines:
        sx, sy = ln["start_x"], ln["start_y"]
        length = ln.get("length", 0.0)
        axis = ln.get("axis", "X")
        ex = sx + length if axis == "X" and ln.get("kind") != "dot" else sx
        ey = sy + length if axis == "Y" and ln.get("kind") != "dot" else sy
        pad = 0.0
        if ln.get("kind") == "line_back" and ln.get("feed_end"):
            pad = _lead(max(ln.get("feed", 0.0), ln["feed_end"]), env, axis)
        if axis == "X":
            xs += [min(sx, ex) - pad, max(sx, ex) + pad]
            ys += [sy, ey]
        else:
            xs += [sx, ex]
            ys += [min(sy, ey) - pad, max(sy, ey) + pad]
    return min(xs), max(xs), min(ys), max(ys)


def migrate(data: dict, env: Envelope) -> tuple[dict, list[str]]:
    lines = data.get("lines", [])
    notes: list[str] = []
    if not lines:
        return data, ["no lines"]

    # 1. feeds. One factor for the whole recipe, so gradient ratios survive.
    top = 0.0
    for ln in lines:
        top = max(top, ln.get("feed", 0.0), ln.get("feed_end", 0.0) or 0.0)
    factor = 1.0
    if top > env.max_feed_xy:
        factor = env.max_feed_xy / top
        notes.append(f"feeds scaled x{factor:.3f} (top was F{top:g}, "
                     f"ceiling is F{env.max_feed_xy:g}) - ratios preserved")
    for ln in lines:
        if ln.get("feed"):
            ln["feed"] = round(ln["feed"] * factor, 1)
        if ln.get("feed_end"):
            ln["feed_end"] = round(ln["feed_end"] * factor, 1)

    # 2. geometry. Flip, then centre what is left.
    lo_x, hi_x, lo_y, hi_y = _extent(lines, env)
    w, h = hi_x - lo_x, hi_y - lo_y
    avail_x, avail_y = env.hi_x - env.lo_x, env.hi_y - env.lo_y
    if w > avail_x:
        notes.append(f"! pattern is {w:.1f} mm wide in X, travel is {avail_x:.1f} mm")
    if h > avail_y:
        notes.append(f"! pattern is {h:.1f} mm tall in Y, travel is {avail_y:.1f} mm")
    off_x = env.lo_x + (avail_x - w) / 2.0 - lo_x
    off_y = env.lo_y + (avail_y - h) / 2.0 - lo_y
    for ln in lines:
        # A pure translation. Both machines run negative from a back-right
        # home, so the only thing that has to change is where the pattern
        # sits - the shape, the spacing and the direction of every pass
        # carry over untouched.
        ln["start_x"] = round(ln["start_x"] + off_x, 3)
        ln["start_y"] = round(ln["start_y"] + off_y, 3)
    notes.append(f"pattern {w:.1f} x {h:.1f} mm placed at "
                 f"X{env.lo_x + (avail_x - w) / 2.0:.1f} "
                 f"Y{env.lo_y + (avail_y - h) / 2.0:.1f}")

    # 3. the cup is a fixture of the old machine; it does not carry over.
    if "cup" in data:
        data["cup"] = None
        notes.append("cup cleared - recapture it on the stage")
    tf = float(data.get("travel_feed", DEFAULT_TRAVEL_FEED) or DEFAULT_TRAVEL_FEED)
    capped = min(tf, DEFAULT_TRAVEL_FEED)
    data["travel_feed"] = f"{capped:g}"
    if capped != tf:
        notes.append(f"travel feed F{tf:g} -> F{capped:g} (the ceiling is a "
                     f"cliff, not a cruising speed)")
    return data, notes


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("files", nargs="+", type=Path)
    ap.add_argument("--out-dir", type=Path, default=None,
                    help="where to write (default: alongside, with a .pico.json suffix)")
    args = ap.parse_args(argv)
    env = Envelope()
    print(f"target envelope  X {env.lo_x:g}..{env.hi_x:g}  Y {env.lo_y:g}..{env.hi_y:g}"
          f"  feed <= {env.max_feed_xy:g}  accel {DEFAULT_ACCEL:g} mm/s2"
          f"  junction {JUNCTION_MMS:g} mm/s")
    for path in args.files:
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError) as e:
            print(f"{path.name}: {e}")
            continue
        data, notes = migrate(data, env)
        if args.out_dir:
            args.out_dir.mkdir(parents=True, exist_ok=True)
            out = args.out_dir / path.name
        else:
            out = path.with_suffix(".pico.json")
        out.write_text(json.dumps(data, indent=2) + "\n")
        print(f"{path.name} -> {out.name}")
        for note in notes:
            print(f"    {note}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
