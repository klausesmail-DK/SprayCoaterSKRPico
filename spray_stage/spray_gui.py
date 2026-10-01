"""Tkinter GUI for masked spray-line testing on the Pico XY stage.

Spray passes are X/Y only. The single exception is the cup / waste position,
which moves Z to establish the spray standoff — see `plan_cup_move`. Nothing
else in this program emits a Z word. See `docs/usage/spray-line-gui.md`.

Run with:
    python -m spray_stage.spray_gui

Coordinates are GRBL machine coordinates, used directly with no translation.
Zero is the home corner — back-right, Z at the top — so every point on the bed
is negative. Directions are conventional: +X right, +Y back, +Z up, which means
travelling away from home makes the numbers more negative.

On this machine the reachable range is X -845…-6, Y -850…-6, Z -90…-6. The
upper bound is the homing pull-off ($27): the head cannot get closer to the
switches than the position it parks at after homing.
"""

from __future__ import annotations

import json
import math
import queue
import re
import threading
import time
import tkinter as tk
from contextlib import suppress
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from tkinter import filedialog, messagebox, simpledialog, ttk

from .pico_controller import PicoController as ShapeokoController
from .pico_controller import PicoError as GrblError
from .pico_controller import find_ports

from .stage import (  # the machine model, importable without Tkinter
    DEFAULT_ACCEL,
    DEFAULT_MAX_FEED_XY,
    DEFAULT_MAX_RATE_XY,
    DEFAULT_MAX_X,
    DEFAULT_MAX_Y,
    DEFAULT_MAX_Z,
    DEFAULT_MIN_X,
    DEFAULT_MIN_Y,
    DEFAULT_PULLOFF,
    DEFAULT_TRAVEL_FEED,
    Envelope,
    GRADIENT_SEGMENT_MM,
    JOG_INVERT_X,
    JOG_INVERT_Y,
    JUNCTION_MMS,
    LEGACY_CUP_FILE,
    MAX_FEED_Z,
    MAX_GRADIENT_SEGMENTS,
    SETTINGS_FILE,
    Z_BOTTOM_MARGIN,
    Z_FEED,
    Z_NOOP_MM,
)


# Line kinds. "line" sprays one way and returns fast; "line_back" sprays in
# both directions; "dot" parks on a point and stays there.
KINDS = {"line": "Line", "line_back": "Line/back", "dot": "Dot"}
KIND_BY_LABEL = {label: key for key, label in KINDS.items()}

# How a Line/back ramps its feed. "velocity" is linear in speed; "thickness" is
# linear in deposited coating, which needs the reciprocal of speed to be linear
# because coating per unit length goes as 1/v.
GRADIENT_MODES = {"velocity": "Velocity", "thickness": "Thickness"}
GRADIENT_MODE_BY_LABEL = {label: key for key, label in GRADIENT_MODES.items()}

FEED_HOLD = 0x21  # '!'
SOFT_RESET = 0x18  # Ctrl-X

# Dark palette. ttk's native macOS theme ignores colour, so the app switches to
# "clam", which honours it, and restyles every widget class it uses.
BG = "#000000"
PANEL = "#141419"
FIELD = "#1c1c24"
BTN = "#26262f"
BTN_HOVER = "#35353f"
FG = "#e8e8ec"
HINT = "#8a8a95"
BORDER = "#3a3a45"
OK_FG = "#4ade80"
BAD_FG = "#ff6b6b"
RING_FILL = ("#16202c", "#22364d", "#2f4f73")
RING_DEAD = ("#141419", "#191920", "#1e1e26")


@dataclass
class SprayLine:
    """One entry in the recipe: a pass, a there-and-back pass, or a dot."""

    kind: str = "line"
    axis: str = "X"
    start_x: float = 20.0
    start_y: float = 20.0
    length: float = 50.0
    feed: float = 900.0
    reps: int = 1
    pause_s: float = 3.0
    dwell_s: float = 1.0  # dots only: how long to stay on the point
    feed_end: float = 0.0  # line/back only: ramp to this feed. 0 = constant
    gradient_mode: str = "velocity"  # how the ramp interpolates: see GRADIENT_MODES

    def end_xy(self) -> tuple[float, float]:
        if self.kind == "dot":
            return self.start_x, self.start_y
        if self.axis == "X":
            return self.start_x + self.length, self.start_y
        return self.start_x, self.start_y + self.length

    @property
    def has_gradient(self) -> bool:
        """True when this pass ramps its feed rather than holding it constant."""
        return self.kind == "line_back" and self.feed_end > 0 and self.feed_end != self.feed

    def validate(self, env: Envelope) -> list[str]:
        errs: list[str] = []
        if self.kind not in KINDS:
            errs.append(f"type must be one of {', '.join(KINDS)}, got {self.kind!r}")
        errs += env.clamp_error("start", self.start_x, self.start_y)
        if self.kind == "dot":
            # Reps, pause, axis, length and feed have no meaning for a dot.
            if self.dwell_s <= 0:
                errs.append("dwell must be greater than zero")
            return errs
        if self.reps < 1:
            errs.append("repetitions must be >= 1")
        if self.pause_s < 0:
            errs.append("pause must be >= 0")
        if self.axis not in ("X", "Y"):
            errs.append(f"axis must be X or Y, got {self.axis!r}")
        if self.length == 0:
            errs.append("length must be non-zero")
        if not 0 < self.feed <= env.max_feed_xy:
            errs.append(f"feed must be in (0, {env.max_feed_xy:g}]")
        if self.has_gradient and not 0 < self.feed_end <= env.max_feed_xy:
            errs.append(f"end feed must be in (0, {env.max_feed_xy:g}]")
        if self.feed_end and self.kind != "line_back":
            errs.append("an end feed only applies to Line/back")
        if self.gradient_mode not in GRADIENT_MODES:
            errs.append(f"gradient mode must be one of {', '.join(GRADIENT_MODES)}")
        errs += env.clamp_error("end", *self.end_xy())
        if self.has_gradient:
            # The pass physically runs from before the start to past the end
            # so its ramps fall outside the coated span. Those run-outs have
            # to fit in travel too, and they are the part nobody remembers.
            lead = env.lead(max(self.feed, self.feed_end), self.axis)
            ex, ey = self.end_xy()
            if self.axis == "X":
                d = 1.0 if ex >= self.start_x else -1.0
                errs += env.clamp_error("lead-in", self.start_x - d * lead, self.start_y)
                errs += env.clamp_error("lead-out", ex + d * lead, ey)
            else:
                d = 1.0 if ey >= self.start_y else -1.0
                errs += env.clamp_error("lead-in", self.start_x, self.start_y - d * lead)
                errs += env.clamp_error("lead-out", ex, ey + d * lead)
            if errs:
                errs.append(
                    f"(a gradient to F{max(self.feed, self.feed_end):g} needs "
                    f"{lead:.2f} mm of run-up at each end)")
        return errs


def jog_target(
    pos: tuple[float, float, float], axis: str, sign: int, step: float
) -> tuple[float, float, float]:
    """Where a jog step would land, in machine coordinates. Used for bounds
    checking before anything is sent."""
    delta = step * sign
    x, y, z = pos
    if axis == "X":
        return x + delta, y, z
    if axis == "Y":
        return x, y + delta, z
    return x, y, z + delta


def jog_to_edge(pos: tuple[float, float, float], axis: str, sign: int, env: Envelope) -> float:
    """Signed distance from `pos` to the envelope edge in the jog direction.

    Hold-to-move sends this as a single long jog and cancels it on release, which
    is GRBL 1.1's intended continuous-jog pattern. Bounding it at the travel
    limit means a dropped release event parks the head at the edge rather than
    driving it into the frame — the machine has no soft limits to fall back on.
    """
    x, y, z = pos
    if axis == "X":
        return (env.hi_x - x) if sign > 0 else (env.lo_x - x)
    if axis == "Y":
        return (env.hi_y - y) if sign > 0 else (env.lo_y - y)
    return (env.hi_z - z) if sign > 0 else (env.lo_z - z)


def plan_cup_move(
    current_z: float, cup: tuple[float, float, float], travel_feed: float
) -> list[str]:
    """G-code to travel to the cup / waste position, in a safe order.

    Retract before traversing, descend after: if the cup sits higher than the
    head is now, Z goes first; if it sits lower, XY goes first. Either way the
    head never crosses the bed lower than it has to.

    This is the only place in the program that commands Z.
    """
    x, y, _z = cup
    # The stage has no Z: the standoff is fixed, so the cup is an XY position
    # and the retract/descend ordering this used to need is gone with it.
    return [f"G90 G1 X{x:.3f} Y{y:.3f} F{travel_feed:g}"]


# Jog pad geometry, in canvas pixels from the centre.
PAD_DEAD = 24.0  # inert centre, so a stray click near the middle does nothing
PAD_RADII = (52.0, 84.0, 116.0)  # outer edge of ring 1, 2, 3
PAD_HOLD_MS = 180  # press longer than this and it becomes a continuous jog


def pad_hit(dx: float, dy: float) -> tuple[str, int, int] | None:
    """Map a click offset from the pad centre to (axis, sign, ring).

    `dx`/`dy` are canvas pixels, so dy grows *downward* while bed +Y is up the
    screen — hence the negation. Returns None for the dead centre and for
    anything outside the outermost ring.
    """
    r = math.hypot(dx, dy)
    if r < PAD_DEAD or r > PAD_RADII[-1]:
        return None
    ring = 1
    for i, edge in enumerate(PAD_RADII, start=1):
        if r <= edge:
            ring = i
            break
    angle = math.degrees(math.atan2(-dy, dx)) % 360.0
    # The pad is a picture of the machine as the operator sees it, so its
    # quadrants name PHYSICAL directions. Which way each one moves the
    # coordinate depends on where the home corner is: homing goes to the
    # back-right, and coordinates count up as the head leaves it, so on this
    # machine "right" and "back" are the DECREASING directions. Flip either
    # constant in stage.py if an axis comes out backwards.
    sx = -1 if JOG_INVERT_X else +1
    sy = -1 if JOG_INVERT_Y else +1
    if angle < 45.0 or angle >= 315.0:
        return "X", +1 * sx, ring  # right
    if angle < 135.0:
        return "Y", +1 * sy, ring  # back, up the screen
    if angle < 225.0:
        return "X", -1 * sx, ring  # left
    return "Y", -1 * sy, ring  # front, down the screen


def gradient_count(length: float) -> int:
    """How many constant-feed segments to approximate a ramp over `length`."""
    return max(2, min(MAX_GRADIENT_SEGMENTS, round(abs(length) / GRADIENT_SEGMENT_MM)))


def gradient_feed(frac: float, feed_start: float, feed_end: float, mode: str) -> float:
    """Feed at fractional position `frac` (0 at the start, 1 at the far end).

    Both modes hit `feed_start` and `feed_end` exactly at the ends; they differ
    only in the curve between.

    - "velocity" interpolates the feed itself, so speed varies linearly. Coating
      thickness then varies as 1/v, which is hyperbolic.
    - "thickness" interpolates 1/feed, so the *coating* varies linearly. That is
      usually what you want from a calibration strip.
    """
    if mode == "thickness":
        inv = (1.0 / feed_start) + ((1.0 / feed_end) - (1.0 / feed_start)) * frac
        return 1.0 / inv
    return feed_start + (feed_end - feed_start) * frac


def gradient_segments(
    start: float,
    end: float,
    feed_start: float,
    feed_end: float,
    segments: int,
    mode: str = "velocity",
) -> list[tuple[float, float]]:
    """(segment end coordinate, feed) for a feed ramp across a pass.

    Feed is sampled at each segment's midpoint, which tracks the intended
    profile better than sampling at either edge.

    Calling this with start/end and both feeds reversed produces the mirrored
    return stroke: every point on the line is then traversed at the same speed
    in both directions, so the ramp shapes the coating rather than smearing it.
    That identity holds for both modes.
    """
    out = []
    for i in range(1, segments + 1):
        pos = start + (end - start) * (i / segments)
        out.append((pos, gradient_feed((i - 0.5) / segments, feed_start, feed_end, mode)))
    return out


def _next_needs_stillness(plan: list[tuple[str, str]], idx: int) -> bool:
    """Must the machine come to rest after the move at `idx`?

    Only if what follows is a wait or the end of the plan. Consecutive moves are
    left to GRBL's look-ahead planner, which blends them into continuous motion
    — essential for feed ramps, where stopping at every segment boundary would
    turn a smooth sweep into a series of lurches.
    """
    for kind, _ in plan[idx + 1 :]:
        if kind == "gcode":
            return False
        if kind in ("pause", "dwell"):
            return True
    return True  # end of the plan


def scale_lines(lines: list[SprayLine], factor: float, max_feed: float) -> list[SprayLine]:
    """Copy `lines` with every spray feed multiplied by `factor`.

    Dwell and pause are durations, not speeds, and are left alone — which means
    scaling changes how much a line deposits but not how much a dot does.

    Feeds are clamped at `max_feed`, so scaling up can never exceed the ceiling.
    """
    if factor == 1.0:
        return list(lines)
    out = []
    for ln in lines:
        out.append(
            replace(
                ln,
                feed=min(ln.feed * factor, max_feed),
                feed_end=min(ln.feed_end * factor, max_feed) if ln.feed_end else 0.0,
            )
        )
    return out


# --------------------------------------------------------------- scripting
#
# Script mode is an alternative front-end to the same flat plan: LOOP/TRANSLATE/
# WAIT expand into exactly the structure sequential mode produces, so every
# coordinate that will actually be sent still goes through the same bounds
# check. That matters because TRANSLATE makes a line's real position depend on
# execution history — validating the entries in the table would no longer say
# anything about where the head goes.


@dataclass
class Stmt:
    """One parsed script statement."""

    op: str  # LOOP | TRANSLATE | WAIT | ORIGIN
    lines: tuple[int, ...] = ()  # LOOP: which entries, 1-based, in order
    count: int = 1  # LOOP: how many times through the group
    dx: float = 0.0  # TRANSLATE
    dy: float = 0.0
    seconds: float = 0.0  # WAIT
    text: str = ""  # the statement as written, for error messages


# Menu entries for the script editor. Each snippet carries real numbers rather
# than placeholders, so an inserted command is valid immediately; the numeric
# part is selected on insert so typing replaces it.
SCRIPT_SNIPPETS: list[tuple[str, str]] = [
    ("LOOP — run entries N times", "LOOP(1,3,5)=3"),
    ("TRANSLATE — shift everything after (cumulative)", "TRANSLATE(10;20)"),
    ("ORIGIN — reset the shift", "ORIGIN"),
    ("WAIT — pause, seconds", "WAIT=3"),
]


def snippet_selection(snippet: str) -> tuple[int, int] | None:
    """Character range of the part worth overtyping — inside the brackets, or
    after the '='. None when there is nothing to fill in."""
    lo = snippet.find("(")
    if lo != -1:
        hi = snippet.find(")")
        return (lo + 1, hi) if hi > lo + 1 else None
    eq = snippet.find("=")
    if eq != -1 and eq + 1 < len(snippet):
        return eq + 1, len(snippet)
    return None


def _split_statements(text: str) -> list[str]:
    """Split on ';' and newlines, but not inside brackets.

    TRANSLATE(10;20) uses ';' as an argument separator too, so a naive split
    would tear that statement in half.
    """
    out, buf, depth = [], [], 0
    for ch in text:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth = max(0, depth - 1)
        if (ch in ";\n") and depth == 0:
            out.append("".join(buf))
            buf = []
        else:
            buf.append(ch)
    out.append("".join(buf))
    return [t.split("#", 1)[0].strip() for t in out]


def _parse_line_list(body: str) -> tuple[int, ...]:
    """`1,3,5` or `1,3-5` -> the entries to run, in the order written."""
    nums: list[int] = []
    for part in re.split(r"[,;]", body):
        part = part.strip()
        if not part:
            continue
        span = re.fullmatch(r"(\d+)\s*-\s*(\d+)", part)
        if span:
            lo, hi = int(span.group(1)), int(span.group(2))
            if lo < 1:
                raise ValueError("entries are numbered from 1")
            if lo > hi:
                raise ValueError(f"range {part!r} runs backwards")
            nums.extend(range(lo, hi + 1))
            continue
        if not part.isdigit():
            raise ValueError(f"{part!r} is not an entry number")
        if int(part) < 1:
            raise ValueError("entries are numbered from 1")
        nums.append(int(part))
    if not nums:
        raise ValueError("no entries listed")
    return tuple(nums)


def parse_script(text: str) -> tuple[list[Stmt], list[str]]:
    """Parse a script. Returns (statements, errors); errors name the statement."""
    stmts: list[Stmt] = []
    errors: list[str] = []
    for raw in _split_statements(text):
        if not raw:
            continue
        head = raw.upper()
        try:
            if head.startswith("LOOP"):
                m = re.fullmatch(r"LOOP\s*\(([^)]*)\)\s*=\s*(\d+)", head)
                if not m:
                    raise ValueError("expected LOOP(1,3,5) = 3")
                count = int(m.group(2))
                if count < 1:
                    raise ValueError("loop count must be at least 1")
                stmts.append(
                    Stmt("LOOP", lines=_parse_line_list(m.group(1)), count=count, text=raw)
                )
            elif head.startswith("TRANSLATE"):
                m = re.fullmatch(r"TRANSLATE\s*\(([^)]*)\)", head)
                if not m:
                    raise ValueError("expected TRANSLATE(10;20)")
                parts = [x for x in re.split(r"[,;]", m.group(1)) if x.strip()]
                if len(parts) != 2:
                    raise ValueError("expected two numbers, X and Y")
                stmts.append(Stmt("TRANSLATE", dx=float(parts[0]), dy=float(parts[1]), text=raw))
            elif head.startswith("WAIT"):
                m = re.fullmatch(r"WAIT\s*=\s*([0-9.]+)", head)
                if not m:
                    raise ValueError("expected WAIT = 3")
                stmts.append(Stmt("WAIT", seconds=float(m.group(1)), text=raw))
            elif head.startswith("ORIGIN"):
                stmts.append(Stmt("ORIGIN", text=raw))
            else:
                raise ValueError("unknown command")
        except ValueError as e:
            errors.append(f"{raw!r}: {e}")
    return stmts, errors


def expand_script(
    lines: list[SprayLine], stmts: list[Stmt]
) -> tuple[list[tuple[str, object, str]], list[str]]:
    """Flatten a script into ('line', SprayLine, context) / ('wait', secs, context).

    TRANSLATE accumulates, and the offset is baked into each entry's start
    position, so what comes out can be bounds-checked directly. The context
    string names the statement and iteration so an error can point at the
    repetition that left the bed rather than just the entry.
    """
    items: list[tuple[str, object, str]] = []
    errors: list[str] = []
    off_x = off_y = 0.0
    for si, st in enumerate(stmts, 1):
        if st.op == "TRANSLATE":
            off_x += st.dx
            off_y += st.dy
            continue
        if st.op == "ORIGIN":
            off_x = off_y = 0.0
            continue
        if st.op == "WAIT":
            items.append(("wait", st.seconds, f"stmt {si} WAIT"))
            continue
        for rep in range(1, st.count + 1):
            for n in st.lines:
                if not 1 <= n <= len(lines):
                    errors.append(f"stmt {si} {st.text!r}: entry {n} does not exist")
                    continue
                src = lines[n - 1]
                moved = replace(src, start_x=src.start_x + off_x, start_y=src.start_y + off_y)
                where = f"stmt {si} loop {rep}/{st.count} entry {n}"
                if off_x or off_y:
                    where += f" (offset {off_x:+g},{off_y:+g})"
                items.append(("line", moved, where))
    return items, errors


def validate_script_items(items: list[tuple[str, object, str]], env: Envelope) -> list[str]:
    """Bounds-check every entry the script will actually execute."""
    errors: list[str] = []
    for kind, payload, where in items:
        if kind != "line":
            continue
        for e in payload.validate(env):  # type: ignore[union-attr]
            errors.append(f"{where}: {e}")
    return errors


def plan_script(
    lines: list[SprayLine], stmts: list[Stmt], travel_feed: float,
    env: Envelope | None = None
) -> list[tuple[str, str]]:
    """Build the run plan for a script. Same shape as `plan_moves`."""
    items, _ = expand_script(lines, stmts)
    plan = list(PREAMBLE)
    for kind, payload, where in items:
        if kind == "wait":
            plan.append(("note", where))
            plan.append(("pause", f"{payload:g}"))
        else:
            plan += plan_line(payload, travel_feed, where, env)  # type: ignore[arg-type]
    return _drop_trailing_pause(plan)


PREAMBLE: list[tuple[str, str]] = [("gcode", "G21"), ("gcode", "G90"), ("gcode", "M5")]


def plan_line(ln: SprayLine, travel_feed: float, label: str,
              env: Envelope | None = None) -> list[tuple[str, str]]:
    """The plan fragment for one entry. `label` prefixes its log note.

    A gradient pass is ONE move per stroke here, not a chain of constant-feed
    segments: the stage sweeps its step rate continuously, so the ramp is
    handed over as a start feed, an end feed and the lead distances, and the
    firmware does the sweeping. The `I` and `K` words carry the lead, and `E`
    the end feed; `Q1` selects thickness mode.
    """
    env = env or Envelope()
    plan: list[tuple[str, str]] = []
    sx, sy = ln.start_x, ln.start_y

    if ln.kind == "dot":
        # A dot is exactly one stay. No repetitions and no drying pause:
        # afterwards it moves straight to the next entry, or to the cup if
        # this was the last one.
        plan.append(("note", f"{label}: dot at {sx:g},{sy:g} — stay {ln.dwell_s:g}s"))
        plan.append(("gcode", f"G1 X{sx:.3f} Y{sy:.3f} F{travel_feed:g}"))
        plan.append(("dwell", f"{ln.dwell_s:g}"))
        return plan

    word = ln.axis
    ex, ey = ln.end_xy()
    far = ex if ln.axis == "X" else ey
    near = sx if ln.axis == "X" else sy
    if ln.has_gradient:
        lead0 = env.lead(max(ln.feed, ln.feed_end), ln.axis)
        d0 = 1.0 if far >= near else -1.0
        if ln.axis == "X":
            sx = sx - d0 * lead0
        else:
            sy = sy - d0 * lead0
    plan.append(
        (
            "note",
            f"{label}: {KINDS[ln.kind]} {ln.axis} {ln.length:g} mm, "
            f"{ln.reps} pass(es), F{ln.feed:g}, pause {ln.pause_s:g}s",
        )
    )
    plan.append(("gcode", f"G1 X{sx:.3f} Y{sy:.3f} F{travel_feed:g}"))
    for r in range(1, ln.reps + 1):
        plan.append(("note", f"  pass {r}/{ln.reps}"))
        if ln.has_gradient:
            q = 1 if ln.gradient_mode == "thickness" else 0
            lead = env.lead(max(ln.feed, ln.feed_end), ln.axis)
            d = 1.0 if far >= near else -1.0
            p0, p1 = near - d * lead, far + d * lead
            # Out and back, both from outside the coated span, so the ramps
            # land off the work. Reversing the feeds on the return makes
            # every point see the same speed in both directions - the
            # gradient shapes the coating instead of smearing it.
            plan.append(("gcode",
                         f"G1 {word}{p1:.3f} F{ln.feed:.0f} "
                         f"E{ln.feed_end:.0f} I{lead:.2f} K{lead:.2f} Q{q}"))
            plan.append(("gcode",
                         f"G1 {word}{p0:.3f} F{ln.feed_end:.0f} "
                         f"E{ln.feed:.0f} I{lead:.2f} K{lead:.2f} Q{q}"))
            if ln.pause_s > 0:
                plan.append(("pause", f"{ln.pause_s:g}"))
            continue
        plan.append(("gcode", f"G1 {word}{far:.3f} F{ln.feed:g}"))
        if ln.kind == "line_back":
            # The return stroke is itself a spray pass, so it runs at the
            # spray feed rather than the fast travel feed.
            plan.append(("gcode", f"G1 {word}{near:.3f} F{ln.feed:g}"))
            if ln.pause_s > 0:
                plan.append(("pause", f"{ln.pause_s:g}"))
        else:
            if ln.pause_s > 0:
                plan.append(("pause", f"{ln.pause_s:g}"))
            if r < ln.reps:
                # Fast one-way return; deposits little because it is quick.
                plan.append(("gcode", f"G1 {word}{near:.3f} F{travel_feed:g}"))
    return plan


def _drop_trailing_pause(plan: list[tuple[str, str]]) -> list[tuple[str, str]]:
    """The last pause has nothing to wait for — the head leaves for the cup."""
    while plan and plan[-1][0] == "pause":
        plan.pop()
    return plan


def plan_moves(lines: list[SprayLine], travel_feed: float,
               env: Envelope | None = None) -> list[tuple[str, str]]:
    """Build the sequential run plan as a list of (kind, payload).

    kind is 'gcode' (send verbatim), 'pause' (drying time — a host-side
    interruptible sleep, payload in seconds), 'dwell' (a dot sitting on its
    point; same mechanism, different meaning), or 'note' (log only).

    The trailing pause is omitted: it would only delay the return to the cup.
    A trailing dwell is kept, because that one is the actual spraying.
    """
    plan = list(PREAMBLE)
    for i, ln in enumerate(lines, 1):
        plan += plan_line(ln, travel_feed, str(i), env)
    return _drop_trailing_pause(plan)


_GWORD = re.compile(r"([XYFEIKQ])(-?\d+(?:\.\d+)?)")


def estimate_seconds(
    plan: list[tuple[str, str]], start: tuple[float, float] | None = None
) -> float:
    """Roughly how long a plan will take, in seconds.

    Distance over feed, plus every pause and dwell. Acceleration is ignored, so
    this runs slightly short — the error is largest on a gradient, where the
    segments are short. Without `start` the opening travel move is skipped,
    since where the head begins is not known at plan time.
    """
    pos = start
    feed = 0.0
    total = 0.0
    for kind, payload in plan:
        if kind in ("pause", "dwell"):
            total += float(payload)
            continue
        if kind != "gcode" or "G1" not in payload:
            continue
        words = dict(_GWORD.findall(payload))
        if "F" in words:
            feed = float(words["F"])
        if not feed:
            continue
        nx = float(words.get("X", pos[0])) if pos else words.get("X")
        ny = float(words.get("Y", pos[1])) if pos else words.get("Y")
        if nx is None or ny is None:
            continue
        nx, ny = float(nx), float(ny)
        if pos is not None:
            dist = math.hypot(nx - pos[0], ny - pos[1])
            total += _move_seconds(dist, feed, words)
        pos = (nx, ny)
    return total


def _move_seconds(dist: float, feed: float, words: dict[str, str]) -> float:
    """Seconds for one move, ramped or not.

    A ramped move is not distance over feed: the speed varies along it, so
    the time is the integral of 1/v. Both modes have a closed form, which is
    worth using - a gradient pass from F750 to F2400 takes about 40% longer
    than its mean feed suggests, and a run estimate that ignores that is
    wrong by minutes over a full pattern.

    Acceleration is ignored, so this still runs slightly short.
    """
    if dist <= 0.0:
        return 0.0
    f1 = float(words.get("E", 0.0))
    if f1 <= 0.0 or abs(f1 - feed) < 1e-6:
        return dist / (feed / 60.0)
    lead_in = float(words.get("I", 0.0))
    lead_out = float(words.get("K", 0.0))
    span = max(0.0, dist - lead_in - lead_out)
    t = lead_in / (feed / 60.0) + lead_out / (f1 / 60.0)
    if int(float(words.get("Q", 0))):                 # thickness: 1/v linear
        t += span * (1.0 / feed + 1.0 / f1) / 2.0 * 60.0
    else:                                             # velocity: v linear
        t += span / (f1 - feed) * math.log(f1 / feed) * 60.0
    return t


def format_duration(seconds: float) -> str:
    if seconds < 60:
        return f"{seconds:.0f}s"
    return f"{int(seconds // 60)}m{int(seconds % 60):02d}s"


def read_envelope(ctrl: ShapeokoController, max_feed_xy: float = DEFAULT_MAX_FEED_XY) -> Envelope:
    """Take the travel, feed and acceleration limits from the stage.

    The transport already parsed `$$` when it connected, so this just copies
    what it found. Acceleration comes across too, because the lead distances
    a gradient needs must match what the firmware will actually do - two
    copies of that constant would drift apart and the coating would quietly
    stop matching the recipe.

    `max_feed_xy` is the operator's own ceiling, clamped by the machine's.
    """
    try:
        lim = ctrl.read_limits()
    except (GrblError, TimeoutError, RuntimeError, AttributeError):
        return Envelope(max_feed_xy=max_feed_xy)
    return Envelope(
        min_x=lim.min_x, max_x=lim.max_x,
        min_y=lim.min_y, max_y=lim.max_y,
        pulloff=abs(lim.max_x),
        max_rate_xy=lim.max_feed,
        max_feed_xy=min(max_feed_xy, lim.max_feed),
        accel_x=lim.accel_x, accel_y=lim.accel_y,
        junction=lim.junction,
    )


class SprayApp(tk.Tk):
    """Single-window controller. Holds one serial connection for its whole life."""

    def __init__(self) -> None:
        super().__init__()
        self._base_title = "Spray Stage"
        self.title(self._base_title)
        self.minsize(900, 480)

        self.ctrl: ShapeokoController | None = None
        self.env = Envelope()
        self.lines: list[SprayLine] = []
        self.homed = False
        self.has_run = False

        self._q: queue.Queue[tuple[str, object]] = queue.Queue()
        self._abort = threading.Event()
        self._worker: threading.Thread | None = None

        # Last known head position, in bed coordinates. None until a status
        # report arrives. Jogging refuses to move without it, because bounds
        # checking is the only thing standing in for the disabled soft limits.
        self.pos: tuple[float, float, float] | None = None
        self._io_lock = threading.Lock()
        self._poll_stop = threading.Event()
        self._poller: threading.Thread | None = None
        self._jogging = False  # a hold-mode jog is in flight
        self.recipe_path: Path | None = None  # file behind Save (vs Save as…)
        self.scripts: dict[str, str] = {}  # named scripts saved with the recipe
        self.ctrl_lines: list[SprayLine] = []  # the control script, run by Ctrl RUN
        self.ctrl_path: Path | None = None
        self._run_t0: float | None = None  # wall-clock start of the current run
        self._drag_iid: str | None = None  # row being dragged to reorder
        self._iid_line: dict[str, SprayLine] = {}
        self._pad_pending: tuple[str, int, int] | None = None
        self._pad_timer: str | None = None

        self._apply_theme()
        self._build_ui()
        self._on_kind_change()
        self._fit_to_screen()
        self._load_settings()
        self._refresh_buttons()
        self._pump_id: str | None = self.after(80, self._pump)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------------------------------------------------------- UI

    def _on_wheel(self, event: tk.Event) -> None:
        """Scroll the page, unless the pointer is over something that scrolls
        itself (the line table or the log)."""
        w = self.winfo_containing(event.x_root, event.y_root)
        while w is not None:
            if isinstance(w, (tk.Text, ttk.Treeview)):
                return
            w = getattr(w, "master", None)
        delta = event.delta
        if abs(delta) >= 120:  # Windows reports multiples of 120; macOS does not
            delta //= 120
        self._canvas.yview_scroll(-delta, "units")

    def _fit_to_screen(self) -> None:
        """Size the window to what it needs, capped to what the screen has."""
        self.update_idletasks()
        pinned = sum(w.winfo_reqheight() + 4 for w in self._pinned)
        want_h = self._body.winfo_reqheight() + pinned + 24
        want_w = max(self._body.winfo_reqwidth() + 20, 900)
        h = min(want_h, self.winfo_screenheight() - 120)
        w = min(want_w, self.winfo_screenwidth() - 60)
        self.geometry(f"{w}x{h}")

    def _set_title(self) -> None:
        name = self.recipe_path.name if self.recipe_path else "untitled"
        self.title(f"{self._base_title} — {name}")

    def _apply_theme(self) -> None:
        """Dark styling. Uses the 'clam' ttk theme because the native macOS
        theme draws widgets with system colours and ignores what we set."""
        self.configure(bg=BG)
        st = ttk.Style(self)
        with suppress(tk.TclError):
            st.theme_use("clam")
        st.configure(".", background=BG, foreground=FG, bordercolor=BORDER, fieldbackground=FIELD)
        st.configure("TFrame", background=BG)
        st.configure("TLabel", background=BG, foreground=FG)
        st.configure("TLabelframe", background=BG, bordercolor=BORDER)
        st.configure("TLabelframe.Label", background=BG, foreground=FG)
        st.configure("TRadiobutton", background=BG, foreground=FG)
        st.map(
            "TRadiobutton",
            background=[("active", BG)],
            indicatorcolor=[("selected", OK_FG), ("!selected", FIELD)],
        )
        st.configure("TButton", background=BTN, foreground=FG, bordercolor=BORDER, focuscolor=BG)
        st.map(
            "TButton",
            background=[("pressed", BTN_HOVER), ("active", BTN_HOVER), ("disabled", PANEL)],
            foreground=[("disabled", HINT)],
        )
        for cls in ("TEntry", "TCombobox"):
            st.configure(
                cls,
                fieldbackground=FIELD,
                foreground=FG,
                background=BTN,
                bordercolor=BORDER,
                insertcolor=FG,
                arrowcolor=FG,
            )
        st.map("TCombobox", fieldbackground=[("readonly", FIELD)])
        st.configure(
            "Treeview",
            background=PANEL,
            fieldbackground=PANEL,
            foreground=FG,
            bordercolor=BORDER,
        )
        st.configure("Treeview.Heading", background=BTN, foreground=FG, bordercolor=BORDER)
        st.map("Treeview", background=[("selected", "#2f4f73")], foreground=[("selected", FG)])
        st.configure("TScrollbar", background=BTN, troughcolor=BG, bordercolor=BORDER)

    def _build_ui(self) -> None:
        pad = {"padx": 6, "pady": 2}

        # Everything lives inside a scrolling canvas: with a full-size jog pad
        # the content is taller than a 800 px screen, and squeezing sections to
        # fit collapsed the line table to a single pixel.
        self._canvas = tk.Canvas(self, bg=BG, highlightthickness=0)
        vsb = ttk.Scrollbar(self, orient="vertical", command=self._canvas.yview)
        self._canvas.configure(yscrollcommand=vsb.set)
        self._vsb = vsb  # packed at the end, after the pinned bottom frames
        body = ttk.Frame(self._canvas)
        window = self._canvas.create_window((0, 0), window=body, anchor="nw")
        body.bind(
            "<Configure>",
            lambda _e: self._canvas.configure(scrollregion=self._canvas.bbox("all")),
        )
        self._canvas.bind(
            "<Configure>", lambda e: self._canvas.itemconfigure(window, width=e.width)
        )
        self.bind_all("<MouseWheel>", self._on_wheel)
        self.bind_all("<Command-s>", self._save)
        self.bind_all("<Control-s>", self._save)
        self._body = body

        # Connection and Home are short and this app is tall, so they share a
        # row.
        top = ttk.Frame(body)
        top.pack(fill="x", **pad)
        conn = ttk.LabelFrame(top, text="1. Connection")
        conn.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.port_var = tk.StringVar()
        ports = find_ports()
        if ports:
            self.port_var.set(ports[0].device)
        self.port_box = ttk.Combobox(
            conn, textvariable=self.port_var, values=[p.device for p in ports], width=32
        )
        self.port_box.pack(side="left", padx=6, pady=6)
        ttk.Button(conn, text="Rescan", command=self._rescan).pack(side="left", padx=3)
        self.btn_connect = ttk.Button(conn, text="Connect", command=self._connect)
        self.btn_connect.pack(side="left", padx=3)
        self.btn_disconnect = ttk.Button(conn, text="Disconnect", command=self._disconnect)
        self.btn_disconnect.pack(side="left", padx=3)
        self.lbl_conn = ttk.Label(conn, text="not connected")
        self.lbl_conn.pack(side="left", padx=12)

        home = ttk.LabelFrame(top, text="2. Home")
        home.pack(side="left", fill="x", expand=True, padx=(4, 0))
        self.btn_home = ttk.Button(home, text="HOME  ($H)", command=self._home)
        self.btn_home.pack(side="left", padx=6, pady=6)
        self.lbl_homed = ttk.Label(home, text="NOT HOMED", foreground=BAD_FG)
        self.lbl_homed.pack(side="left", padx=10)
        ttk.Label(home, text="Required before anything can move.", foreground=HINT).pack(
            side="left", padx=8
        )

        self._build_jog(pad)
        self._build_cup(pad)

        ed = ttk.LabelFrame(body, text="5. Lines  (machine coords: 0,0 at home, work area negative)")
        ed.pack(fill="both", expand=True, **pad)

        form = ttk.Frame(ed)
        form.pack(fill="x", padx=6, pady=6)
        self.f_kind = tk.StringVar(value=KINDS["line"])
        self.f_axis = tk.StringVar(value="X")
        self.f_sx = tk.StringVar(value="20")
        self.f_sy = tk.StringVar(value="20")
        self.f_len = tk.StringVar(value="50")
        self.f_feed = tk.StringVar(value="1000")
        self.f_reps = tk.StringVar(value="1")
        self.f_pause = tk.StringVar(value="3")
        self.f_dwell = tk.StringVar(value="1")
        self.f_feedend = tk.StringVar(value="0")
        self.f_gradmode = tk.StringVar(value=GRADIENT_MODES["velocity"])

        fields = [
            (
                "Type",
                ttk.Combobox(
                    form,
                    textvariable=self.f_kind,
                    values=list(KINDS.values()),
                    state="readonly",
                    width=10,
                ),
            ),
            ("Axis", ttk.Combobox(form, textvariable=self.f_axis, values=["X", "Y"], width=4)),
            ("Start X", ttk.Entry(form, textvariable=self.f_sx, width=8)),
            ("Start Y", ttk.Entry(form, textvariable=self.f_sy, width=8)),
            ("Length", ttk.Entry(form, textvariable=self.f_len, width=8)),
            ("Feed mm/min", ttk.Entry(form, textvariable=self.f_feed, width=9)),
            ("Reps", ttk.Entry(form, textvariable=self.f_reps, width=6)),
            ("Pause s", ttk.Entry(form, textvariable=self.f_pause, width=7)),
            ("Dwell s", ttk.Entry(form, textvariable=self.f_dwell, width=9)),
            ("End feed", ttk.Entry(form, textvariable=self.f_feedend, width=9)),
            (
                "Gradient",
                ttk.Combobox(
                    form,
                    textvariable=self.f_gradmode,
                    values=list(GRADIENT_MODES.values()),
                    state="readonly",
                    width=10,
                ),
            ),
        ]
        self._field_w: dict[str, tk.Widget] = {}
        for col, (label, widget) in enumerate(fields):
            ttk.Label(form, text=label).grid(row=0, column=col, sticky="w", padx=3)
            widget.grid(row=1, column=col, sticky="w", padx=3)
            self._field_w[label] = widget
        self.f_kind.trace_add("write", self._on_kind_change)

        btns = ttk.Frame(ed)
        btns.pack(fill="x", padx=6)
        ttk.Button(btns, text="Add line", command=self._add_line).pack(side="left", padx=3)
        ttk.Button(btns, text="Update selected", command=self._update_line).pack(
            side="left", padx=3
        )
        ttk.Button(btns, text="Remove selected", command=self._remove_line).pack(
            side="left", padx=3
        )
        ttk.Button(btns, text="Duplicate", command=self._duplicate_line).pack(side="left", padx=3)
        ttk.Separator(btns, orient="vertical").pack(side="left", fill="y", padx=8)
        self.btn_cap_start = ttk.Button(
            btns, text="Set start from position", command=self._capture_start
        )
        self.btn_cap_start.pack(side="left", padx=3)
        self.btn_cap_end = ttk.Button(
            btns, text="Set length from position", command=self._capture_length
        )
        self.btn_cap_end.pack(side="left", padx=3)
        ttk.Button(btns, text="Save as…", command=self._save_as).pack(side="right", padx=3)
        ttk.Button(btns, text="Save", command=self._save).pack(side="right", padx=3)
        ttk.Button(btns, text="Load…", command=self._load).pack(side="right", padx=3)

        cols = ("n", "kind", "axis", "start", "length", "end", "feed", "reps", "pause", "dwell")
        # The list gets long, so it scrolls inside a fixed-height table rather
        # than growing the window.
        tree_wrap = ttk.Frame(ed)
        tree_wrap.pack(fill="both", expand=True, padx=6, pady=4)
        self.tree = ttk.Treeview(tree_wrap, columns=cols, show="headings", height=14)
        headings = {
            "n": "#",
            "kind": "Type",
            "axis": "Axis",
            "start": "Start X,Y",
            "length": "Length",
            "end": "End X,Y",
            "feed": "Feed",
            "reps": "Reps",
            "pause": "Pause s",
            "dwell": "Dwell s",
        }
        for c in cols:
            self.tree.heading(c, text=headings[c])
            self.tree.column(c, width=110, anchor="center")
        self.tree.pack(side="left", fill="both", expand=True)
        tsb = ttk.Scrollbar(tree_wrap, orient="vertical", command=self.tree.yview)
        tsb.pack(side="right", fill="y")
        self.tree.configure(yscrollcommand=tsb.set)
        self.tree.bind("<<TreeviewSelect>>", self._on_select)
        # Drag a row to reorder the recipe.
        self.tree.bind("<ButtonPress-1>", self._drag_start, add="+")
        self.tree.bind("<B1-Motion>", self._drag_motion, add="+")
        self.tree.bind("<ButtonRelease-1>", self._drag_end, add="+")

        # Script mode: an alternative to running the table top to bottom.
        sc = ttk.Frame(ed)
        sc.pack(fill="x", padx=6, pady=(4, 0))
        self.run_mode = tk.StringVar(value="sequential")
        ttk.Radiobutton(
            sc,
            text="Sequential",
            value="sequential",
            variable=self.run_mode,
            command=self._on_mode_change,
        ).pack(side="left")
        ttk.Radiobutton(
            sc,
            text="Script",
            value="script",
            variable=self.run_mode,
            command=self._on_mode_change,
        ).pack(side="left", padx=(6, 14))

        ttk.Label(sc, text="Script").pack(side="left", padx=(0, 4))
        self.script_name = tk.StringVar()
        self.script_box = ttk.Combobox(
            sc, textvariable=self.script_name, state="readonly", width=18, values=[]
        )
        self.script_box.pack(side="left")
        self.script_box.bind("<<ComboboxSelected>>", self._on_script_selected)
        ttk.Button(sc, text="New", width=5, command=self._script_new).pack(side="left", padx=2)
        ttk.Button(sc, text="Rename", width=8, command=self._script_rename).pack(side="left")
        ttk.Button(sc, text="Delete", width=7, command=self._script_delete).pack(side="left")
        ttk.Button(sc, text="Insert ▾", width=9, command=self._popup_script_menu).pack(
            side="left", padx=(10, 0)
        )
        self.lbl_script = ttk.Label(sc, text="", foreground=HINT)
        self.lbl_script.pack(side="left", padx=10)

        self.script_text = tk.Text(
            ed,
            height=4,
            wrap="word",
            bg=FIELD,
            fg=FG,
            insertbackground=FG,
            highlightthickness=0,
            relief="flat",
        )
        self.script_text.pack(fill="x", padx=6, pady=(2, 6))
        self.script_text.bind("<KeyRelease>", self._on_script_edited)
        self._script_menu = self._build_script_menu()
        # Right-click reports as Button-2 or Button-3 depending on the platform
        # and mouse; Control-click is the macOS trackpad equivalent.
        for seq in ("<Button-2>", "<Button-3>", "<Control-Button-1>"):
            self.script_text.bind(seq, self._popup_script_menu)

        run = ttk.LabelFrame(self, text="6. Run")

        top_row = ttk.Frame(run)
        top_row.pack(fill="x", padx=4, pady=(4, 0))
        ttk.Label(top_row, text="Max XY feed").pack(side="left", padx=(2, 2))
        self.f_maxfeed = tk.StringVar(value=f"{DEFAULT_MAX_FEED_XY:g}")
        maxfeed = ttk.Entry(top_row, textvariable=self.f_maxfeed, width=7)
        maxfeed.pack(side="left")
        maxfeed.bind("<FocusOut>", self._apply_max_feed)
        maxfeed.bind("<Return>", self._apply_max_feed)
        ttk.Label(top_row, text="Travel / return feed").pack(side="left", padx=(12, 2))
        self.f_travel = tk.StringVar(value="1800")
        ttk.Entry(top_row, textvariable=self.f_travel, width=8).pack(side="left")

        ttk.Label(top_row, text="Speed scale").pack(side="left", padx=(20, 4))
        # Held as log2 so the travel is symmetric about 1.0: -2 is a quarter
        # speed, +2 is four times, and the centre detent is exactly 1.
        self.speed_exp = tk.DoubleVar(value=0.0)
        ttk.Scale(
            top_row,
            from_=-2.0,
            to=2.0,
            variable=self.speed_exp,
            command=self._on_speed_scale,
            length=170,
        ).pack(side="left")
        self.lbl_scale = ttk.Label(top_row, text="1.00×", font=("TkFixedFont", 13), width=7)
        self.lbl_scale.pack(side="left", padx=4)
        ttk.Button(top_row, text="1×", width=3, command=self._reset_speed_scale).pack(side="left")

        btn_row = ttk.Frame(run)
        btn_row.pack(fill="x", padx=4, pady=(2, 4))
        self.btn_run = ttk.Button(btn_row, text="RUN", command=self._run)
        self.btn_run.pack(side="left", padx=4)
        self.btn_stop = ttk.Button(btn_row, text="STOP", command=self._stop)
        self.btn_stop.pack(side="left", padx=4)
        self.lbl_state = ttk.Label(btn_row, text="idle")
        self.lbl_state.pack(side="left", padx=12)
        self.lbl_elapsed = ttk.Label(btn_row, text="", font=("TkFixedFont", 13))
        self.lbl_elapsed.pack(side="left", padx=8)

        # A second, independent recipe — a check pattern you can fire without
        # disturbing whatever is loaded in section 5.
        ctrl_row = ttk.Frame(run)
        ctrl_row.pack(fill="x", padx=4, pady=(0, 4))
        self.btn_ctrl_run = ttk.Button(ctrl_row, text="Ctrl RUN", command=self._ctrl_run)
        self.btn_ctrl_run.pack(side="left", padx=(2, 6))
        self.btn_ctrl_save = ttk.Button(ctrl_row, text="Save", width=6, command=self._ctrl_save)
        self.btn_ctrl_save.pack(side="left", padx=2)
        ttk.Button(ctrl_row, text="Load…", width=7, command=self._ctrl_load).pack(
            side="left", padx=2
        )
        self.lbl_ctrl = ttk.Label(ctrl_row, text="no control script", foreground=HINT)
        self.lbl_ctrl.pack(side="left", padx=10)

        logf = ttk.LabelFrame(self, text="Log")
        self.log = tk.Text(
            logf,
            height=3,
            wrap="none",
            bg=PANEL,
            fg=FG,
            insertbackground=FG,
            highlightthickness=0,
            relief="flat",
        )
        self.log.pack(fill="both", expand=True, side="left", padx=(6, 0), pady=6)
        sb = ttk.Scrollbar(logf, command=self.log.yview)
        sb.pack(fill="y", side="right", padx=(0, 6), pady=6)
        self.log.configure(yscrollcommand=sb.set)

        # Run and the Log are pinned to the bottom of the window, outside the
        # scrolling area, so they stay reachable however tall the rest grows.
        # Log packs first: with side="bottom" the first frame packed sits
        # lowest, so Run ends up above it.
        logf.pack(side="bottom", fill="x", **pad)
        run.pack(side="bottom", fill="x", **pad)
        self._pinned = (run, logf)
        self._vsb.pack(side="right", fill="y")
        self._canvas.pack(side="left", fill="both", expand=True)

    def _build_jog(self, pad: dict) -> None:
        """Jog pad, laid out as a top-down map of the bed.

        Zero is the home corner at back-right, so the bed lies in negative X
        and Y. Directions are conventional: +X right, +Y back.
        """
        jog = ttk.LabelFrame(self._body, text="3. Jog and capture")
        jog.pack(fill="x", **pad)

        side = int(PAD_RADII[-1] * 2 + 24)
        self.pad = tk.Canvas(jog, width=side, height=side, highlightthickness=0, bg=BG)
        self.pad.pack(side="left", padx=10, pady=6)
        self._pad_centre = (side / 2, side / 2)
        self.pad.bind("<ButtonPress-1>", self._pad_press)
        self.pad.bind("<ButtonRelease-1>", self._pad_release)

        rings = ttk.Frame(jog)
        rings.pack(side="left", padx=14)
        ttk.Label(rings, text="Ring").grid(row=0, column=0, padx=4)
        ttk.Label(rings, text="Speed mm/min").grid(row=0, column=1, padx=4)
        ttk.Label(rings, text="Click step mm").grid(row=0, column=2, padx=4)
        self.ring_speed = [tk.StringVar(value=v) for v in ("300", "900", "1800")]
        self.ring_step = [tk.StringVar(value=v) for v in ("0.1", "1", "10")]
        for i, name in enumerate(("inner", "middle", "outer")):
            ttk.Label(rings, text=name).grid(row=i + 1, column=0, sticky="w", padx=4)
            ttk.Entry(rings, textvariable=self.ring_speed[i], width=8).grid(row=i + 1, column=1)
            ttk.Entry(rings, textvariable=self.ring_step[i], width=8).grid(row=i + 1, column=2)
        ttk.Label(
            rings,
            text="Tap = one step.  Hold = move until released.",
            foreground=HINT,
        ).grid(row=4, column=0, columnspan=3, pady=(6, 0))

        # No Z column: the spray standoff is fixed and there is no third
        # motor to move. The variables survive without widgets so that the
        # cup record, the position readout and _z_values() still have the
        # three axes they were written against, with Z pinned at zero.
        self.z_speed = tk.StringVar(value="300")
        self.z_step = tk.StringVar(value="1")

        readout = ttk.Frame(jog)
        readout.pack(side="left", padx=16)
        ttk.Label(readout, text="Position (mm, machine coords)").pack(anchor="w")
        self.lbl_pos = ttk.Label(readout, text="—", font=("TkFixedFont", 15))
        self.lbl_pos.pack(anchor="w")
        self.btn_jogcancel = ttk.Button(readout, text="Cancel jog", command=self._jog_cancel)
        self.btn_jogcancel.pack(anchor="w", pady=4)

    # ----------------------------------------------------------- plumbing

    def _emit(self, kind: str, payload: object = "") -> None:
        self._q.put((kind, payload))

    def _pump(self) -> None:
        try:
            while True:
                kind, payload = self._q.get_nowait()
                if kind == "log":
                    self.log.insert("end", f"{payload}\n")
                    self.log.see("end")
                elif kind == "state":
                    self.lbl_state.configure(text=str(payload))
                elif kind == "conn":
                    self.lbl_conn.configure(text=str(payload))
                elif kind == "homed":
                    self.homed = bool(payload)
                    self.lbl_homed.configure(
                        text="HOMED" if self.homed else "NOT HOMED",
                        foreground=OK_FG if self.homed else BAD_FG,
                    )
                elif kind == "pos":
                    x, y, z = payload  # type: ignore[misc]
                    self.pos = (x, y, z)
                    self.lbl_pos.configure(text=f"X {x:8.2f}   Y {y:8.2f}")
                elif kind == "elapsed":
                    self.lbl_elapsed.configure(text=str(payload))
                elif kind == "buttons":
                    self._refresh_buttons()
        except queue.Empty:
            pass
        if self._run_t0 is not None:
            self.lbl_elapsed.configure(text=f"{time.monotonic() - self._run_t0:6.1f} s")
        self._pump_id = self.after(80, self._pump)

    def _busy(self) -> bool:
        return self._worker is not None and self._worker.is_alive()

    def _refresh_buttons(self) -> None:
        connected = self.ctrl is not None and self.ctrl.is_open
        busy = self._busy()
        self.btn_connect.configure(state="disabled" if connected or busy else "normal")
        self.btn_disconnect.configure(state="normal" if connected and not busy else "disabled")
        self.btn_home.configure(state="normal" if connected and not busy else "disabled")
        self.btn_run.configure(
            state="normal" if connected and self.homed and not busy and self.lines else "disabled"
        )
        self.btn_stop.configure(state="normal" if busy else "disabled")
        self.btn_ctrl_run.configure(
            state="normal"
            if (connected and self.homed and not busy and self.ctrl_lines)
            else "disabled"
        )
        self.btn_ctrl_save.configure(state="disabled" if busy else "normal")
        # Jogging needs a reference: without homing there is no position to
        # bounds-check against, and soft limits are off on this machine.
        self._draw_pad(connected and self.homed and not busy)
        self.btn_jogcancel.configure(state="normal" if connected else "disabled")
        self.btn_cup_go.configure(
            state="normal" if connected and self.homed and not busy else "disabled"
        )
        self.btn_cup_set.configure(state="disabled" if busy else "normal")
        can_jog = "normal" if connected and self.homed and not busy else "disabled"

    def _spawn(self, fn) -> None:
        self._abort.clear()
        self._worker = threading.Thread(target=fn, daemon=True)
        self._worker.start()
        self._refresh_buttons()

    def _build_cup(self, pad: dict) -> None:
        """Cup / waste position. The only part of the program that moves Z."""
        cup = ttk.LabelFrame(self._body, text="4. Cup / waste position")
        cup.pack(fill="x", **pad)

        self.cup_x = tk.StringVar(value="")
        self.cup_y = tk.StringVar(value="")
        self.cup_z = tk.StringVar(value="")
        for label, var in (("X", self.cup_x), ("Y", self.cup_y)):
            ttk.Label(cup, text=label).pack(side="left", padx=(8, 2), pady=6)
            ttk.Entry(cup, textvariable=var, width=8).pack(side="left")

        self.btn_cup_set = ttk.Button(
            cup, text="Set from current position", command=self._capture_cup
        )
        self.btn_cup_set.pack(side="left", padx=10)
        self.btn_cup_go = ttk.Button(cup, text="GO TO CUP", command=self._goto_cup)
        self.btn_cup_go.pack(side="left", padx=4)
        ttk.Button(cup, text="Save cup position", command=self._save_cup).pack(side="left", padx=10)

    def _cup_values(self, quiet: bool = False) -> tuple[float, float, float] | None:
        """Parse and range-check the cup fields. `quiet` skips the dialogs, for
        callers that are not a direct button press."""
        try:
            x, y, z = (float(v.get()) for v in (self.cup_x, self.cup_y, self.cup_z))
        except ValueError:
            if not quiet:
                messagebox.showerror("Bad value", "Cup X, Y and Z must all be numbers.")
            return None
        errs = self.env.clamp_error("cup", x, y) + self.env.clamp_error_z("cup", z)
        if errs:
            if not quiet:
                messagebox.showerror("Cup out of range", "\n".join(errs))
            return None
        return x, y, z

    def _write_settings(self, cup: tuple[float, float, float] | None) -> bool:
        """Persist everything the operator dials in and expects to find again.

        The cup, the feed ceiling, the travel feed, the jog rings and which
        recipe was open. These are properties of the bench, not of the job:
        once the parameters are right they should be right tomorrow too,
        without being retyped.

        The speed scale is deliberately NOT saved. It is a temporary
        override, and restoring a 4x scale into a fresh session - where
        nobody remembers setting it - would spray the next pattern at four
        times the intended rate.
        """
        # Merge over what is already stored. This function is called whenever
        # any one setting changes, and rewriting the file wholesale would drop
        # the others whenever their field happened to be blank or invalid.
        data: dict = {}
        try:
            existing = json.loads(SETTINGS_FILE.read_text(encoding="utf-8"))
            if isinstance(existing, dict):
                data = existing
        except (OSError, ValueError):
            pass
        data["max_feed_xy"] = self.env.max_feed_xy
        data["travel_feed"] = self.f_travel.get()
        data["ring_speed"] = [v.get() for v in self.ring_speed]
        data["ring_step"] = [v.get() for v in self.ring_step]
        if cup is not None:
            data["cup"] = list(cup)
        if self.ctrl_path is not None:
            data["ctrl_path"] = str(self.ctrl_path)
        if self.recipe_path is not None:
            data["last_recipe"] = str(self.recipe_path)
        try:
            SETTINGS_FILE.write_text(json.dumps(data, indent=2), encoding="utf-8")
        except OSError as e:
            messagebox.showerror("Save failed", str(e))
            return False
        return True

    def _save_cup(self) -> None:
        cup = self._cup_values()
        if cup is None:
            return
        if self._write_settings(cup):
            self._emit("log", f"cup position saved to {SETTINGS_FILE}")

    def _load_settings(self) -> None:
        """Restore saved settings, falling back to the pre-settings cup file."""
        data: dict = {}
        for path in (SETTINGS_FILE, LEGACY_CUP_FILE):
            try:
                loaded = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            if isinstance(loaded, dict):
                data = loaded
                break

        try:
            feed = float(data["max_feed_xy"])
        except (KeyError, TypeError, ValueError):
            feed = DEFAULT_MAX_FEED_XY
        if feed > DEFAULT_MAX_FEED_XY:
            # Settings saved against the Shapeoko carry its 6000 mm/min
            # ceiling. Keeping it would fail every jog and every run against
            # a limit the operator never chose.
            self._emit("log", f"saved max feed F{feed:g} is above this "
                              f"machine's F{DEFAULT_MAX_FEED_XY:g} - clamped")
            feed = DEFAULT_MAX_FEED_XY
        self.env.max_feed_xy = feed
        self.f_maxfeed.set(f"{feed:g}")
        try:
            tf = float(data["travel_feed"])
            self.f_travel.set(f"{min(tf, feed):g}")
        except (KeyError, TypeError, ValueError):
            pass

        for key, group in (("ring_speed", self.ring_speed),
                           ("ring_step", self.ring_step)):
            saved = data.get(key)
            if isinstance(saved, list) and len(saved) == len(group):
                for var, val in zip(group, saved):
                    var.set(str(val))

        for i, var in enumerate(self.ring_speed):
            try:
                if float(var.get()) > feed:
                    var.set(f"{feed:g}")
            except ValueError:
                pass

        try:
            x, y, _z = data["cup"]
            x, y = float(x), float(y)
            errs = self.env.clamp_error("stored cup", x, y)
            if errs:
                # A cup saved against another machine, or against the
                # positive-coordinate build. Say so rather than restoring a
                # position that will be refused the moment it is used.
                self._emit("log", f"! stored cup X{x:g} Y{y:g} is outside this "
                                  f"machine's travel - recapture it")
            else:
                self.cup_x.set(f"{x:.2f}")
                self.cup_y.set(f"{y:.2f}")
                self.cup_z.set("0.00")
                self._emit("log", f"cup position restored: X{x:.2f} Y{y:.2f}")
        except (KeyError, TypeError, ValueError):
            pass

        # Quietly: a control script that has been moved or deleted should not
        # throw a dialog at startup.
        saved = data.get("ctrl_path")
        if saved:
            self._ctrl_load(Path(saved), quiet=True)
        self._set_ctrl_label()

        # Reopen whatever was open last time. In practice the same pattern
        # gets run over and over while a paint is being dialled in, so
        # starting empty means hunting through a file dialog every session
        # for the file that was already there.
        last = data.get("last_recipe")
        if last:
            self._load_recipe(Path(last), quiet=True)

    def _apply_max_feed(self, *_args: object) -> None:
        """Validate the XY feed ceiling, apply it, and save it.

        The firmware maximum ($110/$111) is the hard bound; the operator setting
        can sit anywhere at or below it.
        """
        try:
            v = float(self.f_maxfeed.get())
        except ValueError:
            return
        if not 0 < v <= self.env.max_rate_xy:
            messagebox.showerror(
                "Bad value",
                f"Max XY feed must be in (0, {self.env.max_rate_xy:g}] — "
                f"the firmware limit from $110/$111.",
            )
            self.f_maxfeed.set(f"{self.env.max_feed_xy:g}")
            return
        if v == self.env.max_feed_xy:
            return
        self.env.max_feed_xy = v
        self._emit("log", f"max XY feed set to {v:g} mm/min")
        self._write_settings(self._cup_values(quiet=True))

    def _capture_cup(self) -> None:
        if self.pos is None:
            messagebox.showinfo("No position", "Connect and home first.")
            return
        x, y, z = self.pos
        self.cup_x.set(f"{x:.2f}")
        self.cup_y.set(f"{y:.2f}")
        self.cup_z.set(f"{z:.2f}")
        self._emit("log", f"cup position set: X{x:.2f} Y{y:.2f} Z{z:.2f}")

    def _send_moves(self, ctrl: ShapeokoController, moves: list[str]) -> bool:
        """Send absolute moves one at a time, waiting for each to finish.

        Returns False if aborted or a move stalled. Caller owns the I/O lock.
        """
        for line in moves:
            if self._abort.is_set():
                self._emit("log", "aborted.")
                return False
            ctrl.send(line, response_timeout=600.0)
            self._emit("log", f"    {line}")
            if not self._wait_done(ctrl, timeout=300.0):
                self._emit("log", "! move did not finish in time — stopping")
                return False
        return True

    def _goto_cup(self) -> None:
        ctrl = self.ctrl
        if ctrl is None or not self.homed or self._busy():
            return
        if self.pos is None:
            self._emit("log", "! no position yet — wait for the readout")
            return
        cup = self._cup_values()
        tf = self._travel_feed()
        if cup is None or tf is None:
            return
        moves = plan_cup_move(self.pos[2], cup, tf)
        order = "Z then XY" if cup[2] >= self.pos[2] else "XY then Z"
        self._emit("log", f"-> cup X{cup[0]:g} Y{cup[1]:g} Z{cup[2]:g}  ({order})")

        def work() -> None:
            self._io_lock.acquire()
            try:
                self._emit("state", "moving to cup…")
                if self._send_moves(ctrl, moves):
                    self._emit("log", "at cup position.")
            except (GrblError, TimeoutError, RuntimeError) as e:
                self._emit("log", f"! {e}")
            finally:
                self._io_lock.release()
                self._emit("state", "idle")
                self._emit("buttons")

        self._spawn(work)

    # ------------------------------------------------------------ jog pad

    def _draw_pad(self, enabled: bool) -> None:
        """Redraw the pad. Greyed out until the machine is homed."""
        cx, cy = self._pad_centre
        self.pad.delete("all")
        fills = RING_FILL if enabled else RING_DEAD
        edge = BORDER if enabled else "#26262c"
        # Largest first: each smaller ring is drawn over the one outside it.
        for idx in (2, 1, 0):
            r = PAD_RADII[idx]
            for start in (315, 45, 135, 225):
                self.pad.create_arc(
                    cx - r,
                    cy - r,
                    cx + r,
                    cy + r,
                    start=start,
                    extent=90,
                    fill=fills[idx],
                    outline=edge,
                    style=tk.PIESLICE,
                )
        self.pad.create_oval(
            cx - PAD_DEAD,
            cy - PAD_DEAD,
            cx + PAD_DEAD,
            cy + PAD_DEAD,
            fill=PANEL if enabled else "#101015",
            outline=edge,
        )
        label = FG if enabled else HINT
        off = (PAD_RADII[-1] + PAD_DEAD) / 2
        for text, dx, dy in (
            ("+Y\nback", 0, -off),
            ("−Y\nfront", 0, off),
            ("+X\nright", off, 0),
            ("−X\nleft", -off, 0),
        ):
            self.pad.create_text(
                cx + dx,
                cy + dy,
                text=text,
                fill=label,
                justify="center",
                font=("TkDefaultFont", 10),
            )
        if not enabled:
            self.pad.create_text(cx, cy, text="home\nfirst", fill=HINT, justify="center")

    def _ring_values(self, ring: int) -> tuple[float, float] | None:
        """(speed, click step) for a ring, or None if either field is unreadable.

        Ring 0 is the Z column, which carries its own speed and step.
        """
        if ring == 0:
            return self._z_values()
        try:
            speed = float(self.ring_speed[ring - 1].get())
            step = float(self.ring_step[ring - 1].get())
        except ValueError:
            messagebox.showerror("Bad value", "Ring speed and step must be numbers.")
            return None
        if not 0 < speed <= self.env.max_feed_xy:
            messagebox.showerror(
                "Bad value", f"Ring speed must be in (0, {self.env.max_feed_xy:g}]."
            )
            return None
        if step <= 0:
            messagebox.showerror("Bad value", "Click step must be greater than zero.")
            return None
        return speed, step

    def _z_press(self, sign: int) -> None:
        """Z arrow pressed. Same tap-versus-hold timing as the pad."""
        self._pad_pending = ("Z", sign, 0)
        self._pad_timer = self.after(PAD_HOLD_MS, self._pad_hold)

    def _z_values(self) -> tuple[float, float] | None:
        try:
            speed = float(self.z_speed.get())
            step = float(self.z_step.get())
        except ValueError:
            messagebox.showerror("Bad value", "Z speed and step must be numbers.")
            return None
        if not 0 < speed <= MAX_FEED_Z:
            messagebox.showerror("Bad value", f"Z speed must be in (0, {MAX_FEED_Z:g}].")
            return None
        if step <= 0:
            messagebox.showerror("Bad value", "Z step must be greater than zero.")
            return None
        return speed, step

    def _pad_press(self, event: tk.Event) -> None:
        """Arm a jog. A continuous move starts only if the button is still held
        after PAD_HOLD_MS, so a quick tap stays a clean single step."""
        cx, cy = self._pad_centre
        hit = pad_hit(event.x - cx, event.y - cy)
        if hit is None:
            return
        self._pad_pending = hit
        self._pad_timer = self.after(PAD_HOLD_MS, self._pad_hold)

    def _pad_hold(self) -> None:
        self._pad_timer = None
        if self._pad_pending is None:
            return
        axis, sign, ring = self._pad_pending
        vals = self._ring_values(ring)
        if vals is None:
            return
        self._jog(axis, sign, feed=vals[0], continuous=True)

    def _pad_release(self, _event: tk.Event | None) -> None:
        if self._pad_timer is not None:
            # Released before the hold threshold: treat it as a tap.
            self.after_cancel(self._pad_timer)
            self._pad_timer = None
            if self._pad_pending is not None:
                axis, sign, ring = self._pad_pending
                vals = self._ring_values(ring)
                if vals is not None:
                    self._jog(axis, sign, feed=vals[0], step=vals[1])
        elif self._jogging:
            self._jog_cancel(quiet=True)
        self._pad_pending = None

    # ------------------------------------------------------------- jogging

    def _start_poller(self) -> None:
        """Background position poll, so the readout tracks the head while jogging."""
        self._poll_stop.clear()

        def work() -> None:
            while not self._poll_stop.is_set():
                ctrl = self.ctrl
                # The job worker owns the port while it runs; it reports
                # position itself from _wait_done.
                if ctrl is None or not ctrl.is_open or self._busy():
                    time.sleep(0.25)
                    continue
                if not self._io_lock.acquire(blocking=False):
                    time.sleep(0.1)
                    continue
                try:
                    st = ctrl.read_status(timeout=0.3)
                except (OSError, RuntimeError):
                    st = None
                finally:
                    self._io_lock.release()
                if st is not None and st.mpos is not None:
                    self._emit("pos", st.mpos)
                time.sleep(0.2)

        self._poller = threading.Thread(target=work, daemon=True)
        self._poller.start()

    def _jog(
        self,
        axis: str,
        sign: int,
        *,
        feed: float,
        step: float = 0.0,
        continuous: bool = False,
    ) -> None:
        """Send a `$J=` jog, after checking it stays inside the envelope."""
        ctrl = self.ctrl
        if ctrl is None or not self.homed or self._busy():
            return
        if self.pos is None:
            self._emit("log", "! no position yet — wait for the readout, then jog")
            return

        if continuous:
            # Run to the bed edge and no further; release cancels it early.
            distance = jog_to_edge(self.pos, axis, sign, self.env)
            if abs(distance) < 0.05:
                lo = self.env.lo_x if axis == "X" else self.env.lo_y
                hi = self.env.hi_x if axis == "X" else self.env.hi_y
                at = self.pos[0] if axis == "X" else self.pos[1]
                self._emit("log", f"! {axis} is at {at:.2f}, the "
                                  f"{'near' if sign < 0 else 'far'} end of "
                                  f"[{lo:g}, {hi:g}] - jog the other way")
                return
        else:
            distance = step * sign
            errs = self.env.clamp_error_xyz("jog target", *jog_target(self.pos, axis, sign, step))
            if errs:
                self._emit("log", f"! jog refused: {'; '.join(errs)}")
                return

        # Bed and machine frames differ by an offset only, so the delta carries over.
        if not self._io_lock.acquire(timeout=1.0):
            self._emit("log", "! port busy, jog skipped")
            return
        try:
            if continuous:
                # Runs on the transport's own thread and stops on release.
                # A blocking send here would hold the lock for the whole
                # traverse, and the release that must cancel it would queue
                # up behind it - the button would appear dead.
                ctrl.start_jog(axis, distance, feed)
                self._jogging = True
            else:
                ctrl.send(f"$J=G91 G21 {axis}{distance:.3f} F{feed:g}",
                          response_timeout=120.0)
        except (GrblError, TimeoutError, RuntimeError) as e:
            self._emit("log", f"! jog failed: {e}")
        finally:
            self._io_lock.release()

    def _jog_cancel(self, quiet: bool = False) -> None:
        """Realtime jog cancel (0x85). Decelerates and flushes the jog, but
        unlike STOP it keeps the home reference.

        Sent without the I/O lock on purpose: realtime bytes are designed to be
        injected mid-stream, and a button release must never wait on a lock.
        """
        ctrl = self.ctrl
        if ctrl is None or not ctrl.is_open:
            return
        self._jogging = False
        try:
            ctrl.send_realtime(0x85)
            if not quiet:
                self._emit("log", "jog cancelled")
        except (OSError, RuntimeError) as e:
            self._emit("log", f"! jog cancel: {e}")

    def _capture_start(self) -> None:
        if self.pos is None:
            messagebox.showinfo("No position", "Connect and home first.")
            return
        x, y, _z = self.pos
        self.f_sx.set(f"{x:.2f}")
        self.f_sy.set(f"{y:.2f}")
        self._emit("log", f"captured start position: X{x:.2f} Y{y:.2f}")

    def _capture_length(self) -> None:
        """Set Length from the current head position relative to the start fields."""
        if self.pos is None:
            messagebox.showinfo("No position", "Connect and home first.")
            return
        try:
            sx, sy = float(self.f_sx.get()), float(self.f_sy.get())
        except ValueError:
            messagebox.showerror("Bad value", "Set a start position first.")
            return
        x, y, _z = self.pos
        axis = self.f_axis.get().strip().upper()
        length = (x - sx) if axis == "X" else (y - sy)
        self.f_len.set(f"{length:.2f}")
        off_axis = abs(y - sy) if axis == "X" else abs(x - sx)
        self._emit("log", f"captured length along {axis}: {length:.2f} mm")
        if off_axis > 0.5:
            self._emit(
                "log",
                f"  note: head is {off_axis:.2f} mm off the {axis} axis from the start "
                f"point; only the {axis} component was used",
            )

    # ---------------------------------------------------------- connection

    def _rescan(self) -> None:
        ports = find_ports()
        self.port_box.configure(values=[p.device for p in ports])
        if ports and not self.port_var.get():
            self.port_var.set(ports[0].device)
        self._emit("log", f"found {len(ports)} port(s): {', '.join(p.device for p in ports)}")

    def _connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("No port", "Select a serial port first.")
            return

        def work() -> None:
            try:
                # Timed, and reported. "It took ages to connect" is not
                # something anyone can act on; "open 0.1 s, handshake 7.4 s"
                # says immediately whether the board was still booting, the
                # port was contested, or nothing answered at all.
                t0 = time.monotonic()
                ctrl = ShapeokoController(port)
                ctrl.connect()
                t_open = time.monotonic() - t0
                self.ctrl = ctrl
                self._emit("log", f"connected to {port}: {ctrl.banner}")
                if t_open > 2.0:
                    self._emit("log", f"! handshake took {t_open:.1f} s - the "
                                      f"board was probably still booting "
                                      f"(reset, or a fresh ./push.sh)")
                ctrl.drain(quiet_seconds=0.15)
                self.env = read_envelope(ctrl, max_feed_xy=self.env.max_feed_xy)
                self._emit(
                    "log",
                    f"envelope from board: X {self.env.lo_x:g}…{self.env.hi_x:g}, "
                    f"Y {self.env.lo_y:g}…{self.env.hi_y:g}",
                )
                self._emit("conn", f"connected: {port}")
                # GRBL reset when the port opened, so connecting always meant
                # losing the home reference. MicroPython does not reset on
                # DTR: if the stage was homed before the GUI was launched, it
                # still is, and it will say so. Forcing a re-home here would
                # throw away a perfectly good reference and make the operator
                # re-establish it for nothing.
                st = ctrl.read_status(timeout=1.0)
                homed = st is not None and st.state != "Alarm"
                self._emit("homed", homed)
                if homed and st.mpos is not None:
                    self._emit("pos", st.mpos)
                    self._emit("log", f"board is still homed at "
                                      f"X{st.mpos[0]:.2f} Y{st.mpos[1]:.2f}")
                else:
                    self._emit("log", "board is not homed - HOME before running")
                self._start_poller()
            except Exception as e:  # noqa: BLE001 — surfaced to the user
                self.ctrl = None
                self._emit("log", f"! connect failed: {e}")
                self._emit("conn", "not connected")
            finally:
                self._emit("buttons")

        self._spawn(work)

    def _disconnect(self) -> None:
        self._poll_stop.set()
        self._jogging = False
        if self.ctrl is not None:
            self.ctrl.close()
            self.ctrl = None
        self.pos = None
        self.lbl_pos.configure(text="—")
        self._emit("conn", "not connected")
        self._emit("homed", False)
        self._emit("log", "disconnected — home reference lost")
        self._refresh_buttons()

    # -------------------------------------------------------------- homing

    def _home(self) -> None:
        if self.ctrl is None:
            return
        if self.has_run and not messagebox.askyesno(
            "Home again?",
            "Homing raises Z to the top, so you will lose the standoff you set by hand.\n\n"
            "Home anyway?",
        ):
            return

        def work() -> None:
            ctrl = self.ctrl
            if ctrl is None:
                return
            self._io_lock.acquire()
            try:
                self._emit("state", "homing…")
                self._emit("log", "-> $H  (X, then the Y gantry, to the back-right corner)")
                t0 = time.monotonic()
                resp = ctrl.send("$H", response_timeout=120.0)
                if resp and resp != "ok":
                    self._emit("log", f"   {resp}")
                for line in ctrl.drain(quiet_seconds=0.5):
                    self._emit("log", f"   > {line}")
                st = ctrl.read_status(timeout=1.5)
                if st is not None and st.state == "Idle":
                    self._emit("log", f"homed in {time.monotonic() - t0:.1f} s — {st.raw}")
                    self._emit("homed", True)
                    self._emit("log", "Now set the Z standoff by hand, then RUN.")
                else:
                    got = st.state if st else "?"
                    self._emit("log", f"! homing did not settle (state={got}); see docs")
                    self._emit("homed", False)
            except (GrblError, TimeoutError, RuntimeError) as e:
                self._emit("log", f"! homing failed: {e}")
                self._emit("homed", False)
            finally:
                self._io_lock.release()
                self._emit("state", "idle")
                self._emit("buttons")

        self._spawn(work)

    # --------------------------------------------------------------- lines

    def _on_kind_change(self, *_args: object) -> None:
        """Grey out the fields the selected type ignores, so a dot cannot be
        given a pause or a repetition count that would silently do nothing."""
        kind = KIND_BY_LABEL.get(self.f_kind.get())
        dot = kind == "dot"
        ramps = kind == "line_back"
        self._field_w["End feed"].configure(state="normal" if ramps else "disabled")
        self._field_w["Gradient"].configure(state="readonly" if ramps else "disabled")
        for name in ("Axis", "Start X", "Start Y", "Length", "Feed mm/min", "Reps", "Pause s"):
            widget = self._field_w.get(name)
            if widget is None:
                continue
            if name in ("Start X", "Start Y"):
                widget.configure(state="normal")
            else:
                widget.configure(state="disabled" if dot else "normal")
        self._field_w["Dwell s"].configure(state="normal" if dot else "disabled")

    def _read_form(self) -> SprayLine | None:
        try:
            ln = SprayLine(
                kind=KIND_BY_LABEL.get(self.f_kind.get(), "line"),
                # End feed belongs to Line/back alone. Reading it for other
                # types would turn a stale value left in the greyed-out box
                # into a validation error you cannot clear from the form.
                feed_end=(
                    float(self.f_feedend.get() or 0)
                    if KIND_BY_LABEL.get(self.f_kind.get()) == "line_back"
                    else 0.0
                ),
                gradient_mode=GRADIENT_MODE_BY_LABEL.get(self.f_gradmode.get(), "velocity"),
                axis=self.f_axis.get().strip().upper(),
                start_x=float(self.f_sx.get()),
                start_y=float(self.f_sy.get()),
                length=float(self.f_len.get()),
                feed=float(self.f_feed.get()),
                reps=int(self.f_reps.get()),
                pause_s=float(self.f_pause.get()),
                dwell_s=float(self.f_dwell.get()),
            )
        except ValueError as e:
            messagebox.showerror("Bad value", f"Could not read the form: {e}")
            return None
        errs = ln.validate(self.env)
        if errs:
            messagebox.showerror("Line out of range", "\n".join(errs))
            return None
        return ln

    def _write_form(self, ln: SprayLine) -> None:
        self.f_kind.set(KINDS.get(ln.kind, KINDS["line"]))
        self.f_axis.set(ln.axis)
        self.f_sx.set(f"{ln.start_x:g}")
        self.f_sy.set(f"{ln.start_y:g}")
        self.f_len.set(f"{ln.length:g}")
        self.f_feed.set(f"{ln.feed:g}")
        self.f_reps.set(str(ln.reps))
        self.f_pause.set(f"{ln.pause_s:g}")
        self.f_dwell.set(f"{ln.dwell_s:g}")
        self.f_feedend.set(f"{ln.feed_end:g}")
        self.f_gradmode.set(GRADIENT_MODES.get(ln.gradient_mode, GRADIENT_MODES["velocity"]))

    def _row_values(self, ln: SprayLine, number: int) -> tuple:
        """One table row. Fields that don't apply to the kind show as em dashes."""
        dot = ln.kind == "dot"
        ex, ey = ln.end_xy()
        return (
            number,
            KINDS.get(ln.kind, ln.kind),
            "—" if dot else ln.axis,
            f"{ln.start_x:g}, {ln.start_y:g}",
            "—" if dot else f"{ln.length:g}",
            "—" if dot else f"{ex:g}, {ey:g}",
            "—"
            if dot
            else (
                f"{ln.feed:g}→{ln.feed_end:g} ({ln.gradient_mode[:3]})"
                if ln.has_gradient
                else f"{ln.feed:g}"
            ),
            "—" if dot else ln.reps,
            "—" if dot else f"{ln.pause_s:g}",
            f"{ln.dwell_s:g}" if dot else "—",
        )

    def _redraw_tree(self, select: int | None = None) -> None:
        """Rebuild the table. Rebuilding drops the selection, so `select` puts
        it back on the given row index."""
        self.tree.delete(*self.tree.get_children())
        self._iid_line = {}
        iids = []
        for ln in self.lines:
            iid = self.tree.insert("", "end", values=self._row_values(ln, len(iids) + 1))
            self._iid_line[iid] = ln
            iids.append(iid)
        if select is not None and 0 <= select < len(iids):
            self.tree.selection_set(iids[select])
        self._refresh_buttons()

    def _selected(self) -> int | None:
        sel = self.tree.selection()
        if not sel:
            return None
        return self.tree.index(sel[0])

    def _add_line(self) -> None:
        ln = self._read_form()
        if ln is not None:
            self.lines.append(ln)
            self._redraw_tree()

    def _update_line(self) -> None:
        i = self._selected()
        if i is None:
            messagebox.showinfo("No selection", "Select a line in the table first.")
            return
        ln = self._read_form()
        if ln is not None:
            self.lines[i] = ln
            self._redraw_tree(select=i)

    def _remove_line(self) -> None:
        i = self._selected()
        if i is None:
            return
        del self.lines[i]
        self._redraw_tree()

    def _duplicate_line(self) -> None:
        i = self._selected()
        if i is None:
            return
        self.lines.insert(i + 1, SprayLine(**asdict(self.lines[i])))
        self._redraw_tree(select=i + 1)

    def _on_select(self, _evt: object) -> None:
        i = self._selected()
        if i is not None:
            self._write_form(self.lines[i])

    # --- drag to reorder -------------------------------------------------

    def _drag_start(self, event: tk.Event) -> None:
        # Only rows are draggable; clicking a heading must not start a drag.
        if self.tree.identify_region(event.x, event.y) != "cell":
            self._drag_iid = None
            return
        self._drag_iid = self.tree.identify_row(event.y) or None

    def _drag_motion(self, event: tk.Event) -> None:
        if self._drag_iid is None:
            return
        target = self.tree.identify_row(event.y)
        if target and target != self._drag_iid:
            self.tree.move(self._drag_iid, "", self.tree.index(target))

    def _drag_end(self, _event: tk.Event) -> None:
        dragged, self._drag_iid = self._drag_iid, None
        if dragged is None:
            return
        order = [i for i in self.tree.get_children("") if i in self._iid_line]
        if len(order) != len(self.lines):
            return
        reordered = [self._iid_line[i] for i in order]
        if reordered == self.lines:
            # A plain click, not a drag. Redrawing here would wipe the
            # selection the user just made, so leave the table alone.
            return
        self.lines = reordered
        self._emit("log", "line order changed")
        self._redraw_tree(select=order.index(dragged))

    def _save(self, *_event: object) -> None:
        """Write to the current recipe file, asking for one only if unset."""
        if self.recipe_path is None:
            self._save_as()
            return
        self._write_recipe(self.recipe_path)

    def _save_as(self, *_event: object) -> None:
        path = filedialog.asksaveasfilename(defaultextension=".json")
        if path:
            self._write_recipe(Path(path))

    def _write_recipe(self, path: Path) -> None:
        data = {
            "travel_feed": self.f_travel.get(),
            "run_mode": self.run_mode.get(),
            "scripts": dict(self.scripts),
            "active_script": self.script_name.get(),
            "cup": [self.cup_x.get(), self.cup_y.get(), self.cup_z.get()],
            "lines": [asdict(x) for x in self.lines],
        }
        try:
            path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        except OSError as e:
            messagebox.showerror("Save failed", str(e))
            return
        self.recipe_path = path
        self._set_title()
        self._emit("log", f"saved {len(self.lines)} line(s) to {path}")

    def _load(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json"), ("All", "*")])
        if path:
            self._load_recipe(Path(path))

    def _load_recipe(self, path: Path, quiet: bool = False) -> bool:
        """Load a recipe. `quiet` is for the one restored at start-up, where
        a file that has since been moved or renamed should be mentioned in
        the log rather than thrown at the operator as a dialog."""
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            self.lines = [SprayLine(**d) for d in data["lines"]]
            # Not the travel feed: that is a bench setting, dialled in once
            # and kept, the same argument as the cup. A recipe still records
            # it, but reading it back would undo the operator's choice every
            # time a pattern was loaded - which is exactly how a value that
            # "will not stay changed" happens.
        except (OSError, ValueError, KeyError, TypeError) as e:
            if quiet:
                self._emit("log", f"! could not reopen {path.name}: {e}")
            else:
                messagebox.showerror("Load failed", str(e))
            return False
        self.scripts = {str(k): str(v) for k, v in (data.get("scripts") or {}).items()}
        self.run_mode.set(data.get("run_mode", "sequential"))
        self._refresh_script_box(select=data.get("active_script"))
        self.recipe_path = path
        self._set_title()
        self._redraw_tree()
        self._emit("log", f"loaded {len(self.lines)} line(s) from {path}")
        # The cup is a fixture of the MACHINE, not of the job: where the
        # waste cup sits has nothing to do with which pattern is being
        # sprayed. Loading a recipe used to overwrite it - and a recipe
        # carrying no cup silently blanked a position that had been
        # captured by hand. It is kept in the settings file and left alone
        # here; recipes still record it, for reference and for older files.
        if data.get("cup"):
            self._emit("log", "(recipe carries a cup position - ignored, "
                              "the stored one is the machine's)")
        return True

    # ----------------------------------------------------------------- run

    @property
    def speed_factor(self) -> float:
        """The speed-scale slider as a plain multiplier."""
        return round(2.0 ** self.speed_exp.get(), 3)

    def _on_speed_scale(self, *_args: object) -> None:
        self.lbl_scale.configure(text=f"{self.speed_factor:.2f}×")

    def _reset_speed_scale(self) -> None:
        self.speed_exp.set(0.0)
        self._on_speed_scale()

    def _scaled_plan(
        self, lines: list[SprayLine] | None = None, stmts: list[Stmt] | None = None
    ) -> tuple[list[tuple[str, str]], float] | None:
        """The run plan with the speed scale applied, plus the scaled travel feed.

        With `stmts` the plan comes from the script; otherwise from `lines`,
        defaulting to the table.
        """
        tf = self._travel_feed()
        if tf is None:
            return None
        factor = self.speed_factor
        scaled = scale_lines(self.lines if lines is None else lines, factor, self.env.max_feed_xy)
        tf = min(tf * factor, self.env.max_feed_xy)
        if factor != 1.0:
            self._emit("log", f"speed scale {factor:.2f}× applied (dwell and pause unchanged)")
        plan = (plan_script(scaled, stmts, tf, self.env) if stmts
                else plan_moves(scaled, tf, self.env))
        return plan, tf

    def _set_ctrl_label(self) -> None:
        if not self.ctrl_lines:
            self.lbl_ctrl.configure(text="no control script", foreground=HINT)
            return
        name = self.ctrl_path.name if self.ctrl_path else "unsaved"
        self.lbl_ctrl.configure(text=f"{name} — {len(self.ctrl_lines)} line(s)", foreground=FG)

    def _ctrl_save(self) -> None:
        """Store what is currently in the table as the control script.

        The table itself is untouched, so you can build a check pattern, park it
        here, and go back to whatever the main recipe was.
        """
        if not self.lines:
            messagebox.showinfo("Nothing to save", "Add lines to the table first.")
            return
        path = self.ctrl_path or Path(filedialog.asksaveasfilename(defaultextension=".json") or "")
        if not path.name:
            return
        data = {"lines": [asdict(x) for x in self.lines]}
        try:
            path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        except OSError as e:
            messagebox.showerror("Save failed", str(e))
            return
        self.ctrl_path = path
        self.ctrl_lines = [SprayLine(**asdict(x)) for x in self.lines]
        self._write_settings(self._cup_values(quiet=True))
        self._set_ctrl_label()
        self._refresh_buttons()
        self._emit("log", f"control script saved: {len(self.ctrl_lines)} line(s) -> {path}")

    def _ctrl_load(self, path: Path | None = None, quiet: bool = False) -> None:
        if path is None:
            chosen = filedialog.askopenfilename(filetypes=[("JSON", "*.json"), ("All", "*")])
            if not chosen:
                return
            path = Path(chosen)
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            lines = [SprayLine(**d) for d in data["lines"]]
        except (OSError, ValueError, KeyError, TypeError) as e:
            if not quiet:
                messagebox.showerror("Load failed", str(e))
            return
        self.ctrl_lines = lines
        self.ctrl_path = path
        self._set_ctrl_label()
        self._refresh_buttons()
        if not quiet:
            # `quiet` is the startup restore, which is already reading what this
            # would write. Any other load is a new choice worth remembering.
            self._write_settings(self._cup_values(quiet=True))
        self._emit("log", f"control script loaded: {len(lines)} line(s) from {path}")

    # ------------------------------------------------------------- scripts

    def _build_script_menu(self) -> tk.Menu:
        menu = tk.Menu(
            self, tearoff=0, bg=PANEL, fg=FG, activebackground=BTN_HOVER, activeforeground=FG
        )
        for label, snippet in SCRIPT_SNIPPETS:
            menu.add_command(
                label=f"{label}     {snippet}",
                command=lambda t=snippet: self._insert_snippet(t),
            )
        return menu

    def _popup_script_menu(self, event: tk.Event | None = None) -> str:
        if event is not None:
            self._script_menu.tk_popup(event.x_root, event.y_root)
        else:
            x = self.script_text.winfo_rootx() + 20
            y = self.script_text.winfo_rooty() + 20
            self._script_menu.tk_popup(x, y)
        return "break"

    def _insert_snippet(self, snippet: str) -> None:
        """Drop a command at the cursor, on its own line, ready to edit."""
        box = self.script_text
        # Go to the end of the current line first. After a previous insert the
        # cursor is parked inside that command's brackets (so typing overwrites
        # the placeholder), and inserting there would nest one command inside
        # another.
        box.mark_set("insert", "insert lineend")
        current = box.get("insert linestart", "insert")
        text = ("" if current.strip() == "" else "\n") + snippet
        box.insert("insert", text)

        span = snippet_selection(snippet)
        if span is not None:
            head = box.index(f"insert - {len(snippet)} chars")
            a, b = span
            box.tag_remove("sel", "1.0", "end")
            box.tag_add("sel", f"{head} + {a} chars", f"{head} + {b} chars")
            box.mark_set("insert", f"{head} + {b} chars")
        box.focus_set()
        self._on_script_edited()

    def _script_source(self) -> str:
        return self.script_text.get("1.0", "end").strip()

    def _on_mode_change(self) -> None:
        self._on_script_edited()
        self._refresh_buttons()

    def _on_script_edited(self, *_event: object) -> None:
        """Re-check the script on every keystroke and report what it expands to.

        A script hides the direct entry-to-motion correspondence, so the
        expansion has to stay visible or you lose the ability to sanity-check
        it before pressing RUN.
        """
        name = self.script_name.get()
        if name:
            self.scripts[name] = self._script_source()
        text = self._script_source()
        if not text:
            self.lbl_script.configure(text="empty", foreground=HINT)
            return
        stmts, errors = parse_script(text)
        if errors:
            self.lbl_script.configure(text=errors[0][:80], foreground=BAD_FG)
            return
        items, exp_errors = expand_script(self.lines, stmts)
        if exp_errors:
            self.lbl_script.configure(text=exp_errors[0][:80], foreground=BAD_FG)
            return
        passes = sum(1 for k, _, _ in items if k == "line")
        waits = sum(1 for k, _, _ in items if k == "wait")
        moves = sum(1 for k, _ in plan_script(self.lines, stmts, 2000, self.env)
                    if k == "gcode")
        bounds = validate_script_items(items, self.env)
        if bounds:
            self.lbl_script.configure(text=bounds[0][:90], foreground=BAD_FG)
            return
        extra = f", {waits} wait(s)" if waits else ""
        # The real travel feed and speed scale are what will run, so estimate
        # with those rather than the placeholder used for the move count.
        eta = ""
        tf = self._travel_feed(quiet=True)
        if tf is not None:
            scaled = scale_lines(self.lines, self.speed_factor, self.env.max_feed_xy)
            real = plan_script(scaled, stmts,
                           min(tf * self.speed_factor, self.env.max_feed_xy), self.env)
            eta = (
                f", ~{format_duration(estimate_seconds(real, self.pos[:2] if self.pos else None))}"
            )
        self.lbl_script.configure(
            text=f"OK — {passes} entr(ies){extra}, {moves} moves{eta}", foreground=OK_FG
        )

    def _refresh_script_box(self, select: str | None = None) -> None:
        names = sorted(self.scripts)
        self.script_box.configure(values=names)
        if select and select in self.scripts:
            self.script_name.set(select)
        elif names and self.script_name.get() not in self.scripts:
            self.script_name.set(names[0])
        elif not names:
            self.script_name.set("")
        self._load_selected_script()

    def _load_selected_script(self) -> None:
        self.script_text.delete("1.0", "end")
        name = self.script_name.get()
        if name in self.scripts:
            self.script_text.insert("1.0", self.scripts[name])
        self._on_script_edited()

    def _on_script_selected(self, *_event: object) -> None:
        self._load_selected_script()

    def _script_new(self) -> None:
        name = simpledialog.askstring("New script", "Name:", parent=self)
        if not name:
            return
        self.scripts[name.strip()] = ""
        self._refresh_script_box(select=name.strip())

    def _script_rename(self) -> None:
        old = self.script_name.get()
        if not old:
            return
        new = simpledialog.askstring("Rename script", "New name:", initialvalue=old, parent=self)
        if not new or new.strip() == old:
            return
        self.scripts[new.strip()] = self.scripts.pop(old, "")
        self._refresh_script_box(select=new.strip())

    def _script_delete(self) -> None:
        name = self.script_name.get()
        if not name:
            return
        if not messagebox.askyesno("Delete script", f"Delete {name!r}?"):
            return
        self.scripts.pop(name, None)
        self._refresh_script_box()

    def _travel_feed(self, quiet: bool = False) -> float | None:
        """`quiet` suppresses the dialogs, for callers that run on every
        keystroke and must not interrupt typing."""
        try:
            v = float(self.f_travel.get())
        except ValueError:
            if not quiet:
                messagebox.showerror("Bad value", "Travel feed must be a number.")
            return None
        if not 0 < v <= self.env.max_feed_xy:
            if not quiet:
                messagebox.showerror(
                    "Bad value", f"Travel feed must be in (0, {self.env.max_feed_xy:g}]."
                )
            return None
        return v

    def _validate_all(self) -> bool:
        problems: list[str] = []
        for i, ln in enumerate(self.lines, 1):
            for e in ln.validate(self.env):
                problems.append(f"line {i}: {e}")
        if problems:
            messagebox.showerror("Cannot run", "\n".join(problems))
            return False
        return True

    def _run(self) -> None:
        if self.run_mode.get() == "script":
            self._run_script()
            return
        if not self._validate_all():
            return
        self._start_run(self.lines, "run")

    def _run_script(self) -> None:
        """Run the selected script against the entries in the table."""
        stmts, errors = parse_script(self._script_source())
        if errors:
            messagebox.showerror("Script error", "\n".join(errors[:10]))
            return
        if not stmts:
            messagebox.showinfo("Empty script", "Nothing to run.")
            return
        items, exp_errors = expand_script(self.lines, stmts)
        if exp_errors:
            messagebox.showerror("Script error", "\n".join(exp_errors[:10]))
            return
        # TRANSLATE makes an entry's real position depend on what ran before it,
        # so the expansion is what gets bounds-checked, not the table.
        bounds = validate_script_items(items, self.env)
        if bounds:
            messagebox.showerror("Script leaves the machine", "\n".join(bounds[:10]))
            return
        self._start_run(None, f"script {self.script_name.get() or 'untitled'}", stmts=stmts)

    def _ctrl_run(self) -> None:
        """Run the stored control script, leaving the main recipe untouched."""
        if not self.ctrl_lines:
            messagebox.showinfo("No control script", "Load or save one first.")
            return
        errs = [e for ln in self.ctrl_lines for e in ln.validate(self.env)]
        if errs:
            messagebox.showerror("Control script out of range", "\n".join(errs[:10]))
            return
        self._start_run(self.ctrl_lines, "control run")

    def _start_run(
        self,
        lines: list[SprayLine] | None,
        label: str,
        stmts: list[Stmt] | None = None,
    ) -> None:
        if self.ctrl is None or not self.homed:
            return
        scaled = self._scaled_plan(lines, stmts)
        if scaled is None:
            return
        plan, tf = scaled
        total = sum(1 for k, _ in plan if k == "gcode")
        count = f"{len(stmts)} statement(s)" if stmts else f"{len(lines or [])} line(s)"
        eta = estimate_seconds(plan, self.pos[:2] if self.pos else None)
        self._emit("log", f"-> {label}: {count}, {total} moves, ~{format_duration(eta)}")
        self._run_t0 = time.monotonic()
        # Read on the main thread; the worker must not touch Tk variables.
        cup = self._cup_values(quiet=True)
        start_z = self.pos[2] if self.pos else None

        def work() -> None:
            ctrl = self.ctrl
            if ctrl is None:
                return
            self.has_run = True
            done = 0
            # Own the port outright; the poller backs off while we hold this.
            self._io_lock.acquire()
            try:
                for idx, (kind, payload) in enumerate(plan):
                    if self._abort.is_set():
                        self._emit("log", "aborted.")
                        break
                    if kind == "note":
                        self._emit("log", str(payload))
                        continue
                    if kind in ("pause", "dwell"):
                        secs = float(payload)
                        verb = "spraying" if kind == "dwell" else "pausing"
                        self._emit("state", f"{verb} {secs:g}s")
                        end = time.monotonic() + secs
                        while time.monotonic() < end and not self._abort.is_set():
                            time.sleep(0.05)
                        continue
                    ctrl.send(str(payload), response_timeout=600.0)
                    done += 1
                    self._emit("state", f"move {done}/{total}")
                    self._emit("log", f"    {payload}")
                    # GRBL acks a line when it enters the planner, not when the
                    # move finishes, and it withholds the ack while the planner
                    # is full — so send() is its own flow control. Waiting for
                    # Idle here would stop the head dead between every segment,
                    # which is exactly what wrecks a gradient. Only wait when
                    # the next step genuinely needs the machine stationary.
                    if not _next_needs_stillness(plan, idx):
                        continue
                    if not self._wait_done(ctrl, timeout=900.0):
                        self._emit("log", "! move did not finish in time — stopping")
                        break
                else:
                    # Time the pattern itself, before the trip back to the cup.
                    spray_s = time.monotonic() - (self._run_t0 or time.monotonic())
                    self._emit("log", f"run complete in {spray_s:.1f} s.")
                    self._park_at_cup(ctrl, cup, start_z, tf)
            except (GrblError, TimeoutError, RuntimeError) as e:
                self._emit("log", f"! {e}")
            finally:
                self._io_lock.release()
                if self._run_t0 is not None:
                    # Not `total`: that name is the move count this closure uses.
                    elapsed = time.monotonic() - self._run_t0
                    self._run_t0 = None
                    self._emit("elapsed", f"{elapsed:.1f} s")
                    self._emit("log", f"total elapsed {elapsed:.1f} s (including return to cup)")
                self._emit("state", "idle")
                self._emit("buttons")

        self._spawn(work)

    def _park_at_cup(
        self,
        ctrl: ShapeokoController,
        cup: tuple[float, float, float] | None,
        start_z: float | None,
        travel_feed: float,
    ) -> None:
        """Return to the cup once a run finishes cleanly.

        Skipped silently-but-logged when no valid cup position is stored, so a
        missing cup never turns a good run into an error.
        """
        if cup is None:
            self._emit("log", "no valid cup position stored — leaving the head where it is")
            return
        # Lines never touch Z, so the height is still whatever the run began at.
        cur_z = start_z if start_z is not None else cup[2]
        self._emit("state", "returning to cup…")
        self._emit("log", f"-> returning to cup X{cup[0]:g} Y{cup[1]:g} Z{cup[2]:g}")
        if self._send_moves(ctrl, plan_cup_move(cur_z, cup, travel_feed)):
            self._emit("log", "at cup position.")

    def _wait_done(self, ctrl: ShapeokoController, timeout: float) -> bool:
        """Confirm the machine is stopped, and refresh the position readout.

        GRBL acked a line when it entered the planner, so this had to poll
        until it had seen the machine move and then go Idle again - with a
        0.6 s grace in case the move was too short to catch mid-flight.

        The stage acks when the move has actually FINISHED. There is no
        planner buffer and nothing in flight, so by the time this is called
        the machine is already Idle and that grace was pure waiting: six
        tenths of a second before every pause, every dot and the end of
        every run. The first Idle report is now conclusive.
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self._abort.is_set():
                return False
            st = ctrl.read_status(timeout=0.3)
            if st is None:
                continue
            if st.mpos is not None:
                self._emit("pos", st.mpos)
            if st.state == "Idle":
                return True
            time.sleep(0.05)
        return False

    def _stop(self) -> None:
        self._abort.set()
        self._jogging = False
        ctrl = self.ctrl
        if ctrl is not None and ctrl.is_open:
            try:
                # Realtime bytes bypass the line protocol by design, so this is
                # safe to send while the worker thread is mid-read.
                ctrl.send_realtime(FEED_HOLD)
                time.sleep(0.1)
                ctrl.send_realtime(SOFT_RESET)
            except Exception as e:  # noqa: BLE001 — best effort during an abort
                self._emit("log", f"! stop: {e}")
        self._emit("log", "STOP — feed hold + soft reset sent. HOME AGAIN before running.")
        self._emit("homed", False)

    def _on_close(self) -> None:
        # Last chance to keep what was dialled in. Everything else that
        # writes settings does so when one field changes; this catches the
        # rest, so quitting is not a way to lose an afternoon's tuning.
        with suppress(Exception):
            self._write_settings(self._cup_values(quiet=True))
        self._abort.set()
        self._poll_stop.set()
        # Cancel the queue pump first: if it fires after destroy() Tk complains
        # about an invalid command on the way out.
        if self._pump_id is not None:
            with suppress(tk.TclError):
                self.after_cancel(self._pump_id)
            self._pump_id = None
        if self.ctrl is not None:
            self.ctrl.close()
        self.destroy()


def main() -> int:
    SprayApp().mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
