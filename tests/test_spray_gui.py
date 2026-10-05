"""Tests for the spray-line planner and validation.

Only the pure logic is covered — instantiating `SprayApp` needs a display,
so the Tk layer is deliberately left untested here.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict

import pytest

from spray_stage.pico_controller import Limits, PicoController
from spray_stage.spray_gui import (
    DEFAULT_MAX_FEED_XY,
    DEFAULT_MAX_RATE_XY,
    DEFAULT_TRAVEL_FEED,
    GRADIENT_MODES,
    KINDS,
    MAX_GRADIENT_SEGMENTS,
    PAD_DEAD,
    PAD_RADII,
    SCRIPT_SNIPPETS,
    Envelope,
    SprayLine,
    _next_needs_stillness,
    estimate_seconds,
    expand_script,
    move_timeouts,
    format_duration,
    gradient_count,
    gradient_feed,
    gradient_segments,
    jog_target,
    jog_to_edge,
    pad_hit,
    parse_script,
    plan_cup_move,
    plan_moves,
    plan_script,
    read_envelope,
    scale_lines,
    snippet_selection,
    validate_script_items,
)

# A roomy test envelope, deliberately the same numbers the Shapeoko suite
# used, so every coordinate the older tests chose still lies inside it and
# only genuinely changed behaviour needed new assertions. The real stage is
# much smaller; Envelope() carries its true limits.
ENV = Envelope(min_x=-845.0, max_x=-6.0, min_y=-850.0, max_y=-6.0,
               max_feed_xy=6000.0, max_rate_xy=10000.0)

# Acceleration decides how much run-up a gradient pass needs, and the
# planner puts that distance outside the coated span. Tests that care about
# where a ramped move actually starts and ends ask for it the same way the
# planner does rather than hard-coding a number.
def lead_for(feed, axis="Y", env=ENV):
    return env.lead(feed, axis)


def gcode_only(plan):
    return [payload for kind, payload in plan if kind == "gcode"]


def test_zero_is_the_home_corner():
    # Home is back-right and the work area lies in negative X and Y - the
    # convention inherited from the Shapeoko. The firmware counts positive
    # millimetres from the same corner; the transport flips the sign, and
    # nothing above it ever sees a positive coordinate.
    assert (ENV.hi_x, ENV.hi_y) == (-6.0, -6.0)
    assert (ENV.lo_x, ENV.lo_y) == (-845.0, -850.0)


def test_there_is_no_z_axis():
    # The spray standoff is fixed and set by hand. Z collapses to a single
    # point so that the three-axis plumbing above it - the readout, the cup
    # record, jog_target - keeps working without knowing the machine is flat.
    assert ENV.lo_z == ENV.hi_z == 0.0
    assert ENV.clamp_error_z("cup", -70.0) == []
    assert ENV.clamp_error_z("cup", 0.0) == []
    assert ENV.clamp_error_z("cup", 999.0) == []


def test_reachable_range_stops_at_the_pulloff():
    # The head parks at -6 after homing and cannot get closer to the switches.
    assert ENV.clamp_error("p", -6.0, -6.0) == []
    assert ENV.clamp_error("p", -5.0, -400.0)  # past the pull-off
    assert ENV.clamp_error("p", -846.0, -400.0)  # past full travel
    assert ENV.clamp_error("p", 10.0, -400.0)  # positive is off the machine


def test_end_xy_follows_axis():
    assert SprayLine(axis="X", start_x=10, start_y=20, length=5).end_xy() == (15, 20)
    assert SprayLine(axis="Y", start_x=10, start_y=20, length=5).end_xy() == (10, 25)


def test_plan_starts_in_mm_absolute_spindle_off():
    plan = gcode_only(plan_moves([SprayLine()], 4000))
    assert plan[:3] == ["G21", "G90", "M5"]


def test_plan_sends_coordinates_untranslated():
    line = SprayLine(axis="X", start_x=-745, start_y=-650, length=40, feed=800, reps=1, pause_s=0)
    plan = gcode_only(plan_moves([line], 4000))
    assert "G1 X-745.000 Y-650.000 F4000" in plan
    # +40 moves toward home (right), so the coordinate becomes less negative.
    assert "G1 X-705.000 F800" in plan


def test_single_pass_has_no_return_move():
    line = SprayLine(axis="X", start_x=-400, start_y=-400, length=40, feed=800, reps=1, pause_s=0)
    plan = gcode_only(plan_moves([line], 4000))
    # Only the travel-in move uses the travel feed; nothing returns afterwards.
    assert sum("F4000" in m for m in plan) == 1


def test_repetitions_return_between_but_not_after_last():
    line = SprayLine(axis="X", start_x=-400, start_y=-400, length=40, feed=800, reps=3, pause_s=0)
    plan = gcode_only(plan_moves([line], 4000))
    spray = [m for m in plan if "F800" in m]
    returns = [m for m in plan if m == "G1 X-400.000 F4000"]
    assert len(spray) == 3
    assert len(returns) == 2  # 3 passes -> 2 returns


def test_pause_emitted_between_repetitions_but_not_after_the_last():
    # 3 passes -> 2 drying pauses; the trailing one is dropped because the head
    # leaves for the cup immediately afterwards.
    line = SprayLine(axis="X", start_x=-400, start_y=-400, length=40, reps=3, pause_s=2.5)
    pauses = [p for kind, p in plan_moves([line], 4000) if kind == "pause"]
    assert pauses == ["2.5", "2.5"]


def test_only_the_final_pause_of_the_whole_run_is_dropped():
    # Two lines, 2 reps each: line 1 keeps both pauses because line 2 follows,
    # line 2 loses only its last.
    lines = [
        SprayLine("line", "X", -400, -400, 40, 800, 2, 3.0),
        SprayLine("line", "Y", -300, -300, 40, 800, 2, 3.0),
    ]
    pauses = [p for kind, p in plan_moves(lines, 4000) if kind == "pause"]
    assert pauses == ["3", "3", "3"]


def test_plan_never_ends_on_a_pause():
    for reps in (1, 2, 5):
        line = SprayLine("line", "X", -400, -400, 40, 800, reps, 3.0)
        plan = plan_moves([line], 4000)
        assert plan[-1][0] != "pause", reps


def test_no_pause_when_zero():
    line = SprayLine(axis="X", start_x=-400, start_y=-400, length=40, reps=2, pause_s=0)
    assert not [p for kind, p in plan_moves([line], 4000) if kind == "pause"]


def test_y_axis_line_moves_only_y():
    line = SprayLine(axis="Y", start_x=-50, start_y=-60, length=25, feed=600, reps=1, pause_s=0)
    plan = gcode_only(plan_moves([line], 4000))
    assert "G1 Y-35.000 F600" in plan


def test_validate_accepts_in_range_line():
    assert SprayLine("line", "X", -400, -200, 40, 800, 2, 3).validate(ENV) == []


def test_validate_accepts_negative_length():
    assert SprayLine("line", "X", -400, -200, -40, 800, 1, 1).validate(ENV) == []


def test_validate_rejects_end_beyond_envelope():
    errs = SprayLine("line", "X", -840, -200, -100, 800, 1, 1).validate(ENV)
    assert any("end X" in e for e in errs)


def test_validate_rejects_positive_coordinate():
    # Positive X is past the home switch; nothing on the machine is positive.
    errs = SprayLine("line", "X", 5, -200, -40, 800, 1, 1).validate(ENV)
    assert any("start X" in e for e in errs)


def test_validate_rejects_reaching_into_the_pulloff():
    # -6 is as close to the switch as the head gets; -5 is inside the pull-off.
    assert SprayLine("line", "X", -40, -200, 34, 800, 1, 1).validate(ENV) == []
    assert any("end X" in e for e in SprayLine("line", "X", -40, -200, 36, 800, 1, 1).validate(ENV))


def test_validate_rejects_y_beyond_envelope():
    errs = SprayLine("line", "Y", -100, -800, -100, 800, 1, 1).validate(ENV)
    assert any("end Y" in e for e in errs)


def test_feed_ceiling_is_the_measured_step_loss_cliff():
    # Not a preference: both axes hold position to a few microns at 2400
    # mm/min and lose about 0.157 mm at 3000. The firmware refuses anything
    # above it rather than clamping, because clamping would flatten the fast
    # end of a gradient and change the coating while appearing to work.
    assert DEFAULT_MAX_FEED_XY == 2400.0
    assert DEFAULT_MAX_RATE_XY == 2400.0
    assert SprayLine("line", "X", -400, -400, 20, 2400).validate(Envelope(
        min_x=-845.0, max_x=-6.0, min_y=-850.0, max_y=-6.0)) == []


def test_travel_feed_default_sits_below_the_cliff():
    # The ceiling is a cliff, not a cruising speed: there is nothing to gain
    # from crossing the bed at the exact rate where step loss begins.
    assert DEFAULT_TRAVEL_FEED == 1800.0
    assert DEFAULT_TRAVEL_FEED < DEFAULT_MAX_FEED_XY


def test_feed_ceiling_is_per_envelope_not_global():
    roomy = dict(min_x=-845.0, max_x=-6.0, min_y=-850.0, max_y=-6.0)
    slow = Envelope(max_feed_xy=2000.0, **roomy)
    fast = Envelope(max_feed_xy=9000.0, **roomy)
    line = SprayLine("line", "X", -400, -400, 20, 5000)
    assert any("feed" in e for e in line.validate(slow))
    assert line.validate(fast) == []


def test_operator_ceiling_can_only_tighten_the_machine_limit():
    # The operator setting is a guard rail below the firmware's, never above
    # it: raising it cannot make the machine faster, only let a recipe ask
    # for a feed the stage will refuse outright.
    assert ENV.max_rate_xy >= ENV.max_feed_xy or ENV.max_feed_xy <= DEFAULT_MAX_FEED_XY


def test_validate_rejects_bad_scalars():
    assert any("length" in e for e in SprayLine("line", "X", -400, -400, 0).validate(ENV))
    assert any("feed" in e for e in SprayLine("line", "X", -400, -400, 10, 99999).validate(ENV))
    assert any(
        "repetitions" in e for e in SprayLine("line", "X", -400, -400, 10, 800, 0).validate(ENV)
    )
    assert any(
        "pause" in e for e in SprayLine("line", "X", -400, -400, 10, 800, 1, -1).validate(ENV)
    )


def test_jog_target_moves_the_right_axis():
    p = (-400.0, -200.0, -50.0)
    assert jog_target(p, "X", +1, 10.0) == (-390.0, -200.0, -50.0)
    assert jog_target(p, "X", -1, 10.0) == (-410.0, -200.0, -50.0)
    assert jog_target(p, "Y", +1, 10.0) == (-400.0, -190.0, -50.0)
    assert jog_target(p, "Y", -1, 10.0) == (-400.0, -210.0, -50.0)
    assert jog_target(p, "Z", +1, 10.0) == (-400.0, -200.0, -40.0)  # up
    assert jog_target(p, "Z", -1, 10.0) == (-400.0, -200.0, -60.0)  # down


def test_jog_target_is_bounds_checkable():
    # A step that would leave the machine must be catchable before it is sent.
    assert ENV.clamp_error_xyz("jog", *jog_target((-10.0, -200.0, 0.0), "X", +1, 10.0))
    assert ENV.clamp_error_xyz("jog", *jog_target((-840.0, -200.0, 0.0), "X", -1, 10.0))
    assert ENV.clamp_error_xyz("jog", *jog_target((-400.0, -10.0, 0.0), "Y", +1, 10.0))
    assert not ENV.clamp_error_xyz("jog", *jog_target((-400.0, -200.0, 0.0), "X", +1, 10.0))
    # A Z step is unreachable from the GUI and bounds-checks as a no-op.
    assert not ENV.clamp_error_xyz("jog", *jog_target((-400.0, -200.0, 0.0), "Z", -1, 10.0))


def test_jog_to_edge_reaches_the_limits():
    # Hold-to-move runs to the edge of the envelope, never past it.
    p = (-400.0, -200.0, -50.0)
    assert jog_to_edge(p, "X", +1, ENV) == 394.0  # -6 - (-400)
    assert jog_to_edge(p, "X", -1, ENV) == -445.0  # -845 - (-400)
    assert jog_to_edge(p, "Y", +1, ENV) == 194.0  # -6 - (-200)
    assert jog_to_edge(p, "Y", -1, ENV) == -650.0  # -850 - (-200)
    assert jog_to_edge((-400.0, -200.0, 0.0), "Z", +1, ENV) == 0.0  # no Z travel
    assert jog_to_edge((-400.0, -200.0, 0.0), "Z", -1, ENV) == 0.0


def test_jog_to_edge_lands_inside_the_envelope():
    # Whatever the start point, start + distance must be a legal position:
    # a dropped button-release parks at the edge rather than in the frame.
    points = [
        (-6.0, -6.0, -6.0),
        (-400.0, -200.0, -50.0),
        (-845.0, -850.0, -90.0),
        (-500.0, -10.0, -12.0),
    ]
    for pos in points:
        for axis in ("X", "Y", "Z"):
            for sign in (+1, -1):
                d = jog_to_edge(pos, axis, sign, ENV)
                landed = jog_target(pos, axis, 1, d)  # d already carries its sign
                assert ENV.clamp_error_xyz("edge", *landed) == [], (pos, axis, sign, landed)


def test_jog_to_edge_is_zero_when_already_at_the_limit():
    assert jog_to_edge((-6.0, -400.0, -50.0), "X", +1, ENV) == 0.0
    assert jog_to_edge((-845.0, -400.0, -50.0), "X", -1, ENV) == 0.0
    assert jog_to_edge((-400.0, -400.0, 0.0), "Z", +1, ENV) == 0.0
    assert jog_to_edge((-400.0, -400.0, 0.0), "Z", -1, ENV) == 0.0


def test_a_cup_height_from_the_old_machine_is_harmless():
    # Stored settings carry a Z from the Shapeoko. With no Z axis there is
    # nothing for it to be outside of, so it is accepted and ignored rather
    # than rejected - the cup is an XY position now.
    assert ENV.clamp_error_z("cup", -70.02) == []


def test_cup_move_is_one_xy_move():
    # The Shapeoko had to retract before traversing and descend afterwards,
    # so that the head never crossed the bed lower than it had to. With a
    # fixed standoff there is no height to get wrong, and the whole
    # ordering problem goes with it.
    moves = plan_cup_move(0.0, (-745.0, -650.0, 0.0), 4000)
    assert moves == ["G90 G1 X-745.000 Y-650.000 F4000"]


def test_cup_move_ignores_any_stored_height():
    # Whatever Z a settings file carries - including one saved against the
    # old machine - the move is the same single traverse.
    for cup_z in (0.0, -30.0, -70.02, -90.0):
        assert plan_cup_move(-70.0, (-745.0, -650.0, cup_z), 4000) == [
            "G90 G1 X-745.000 Y-650.000 F4000"
        ]


def test_cup_move_sends_coordinates_untranslated():
    moves = plan_cup_move(0.0, (-745.0, -650.0, 0.0), 4000)
    assert any("X-745.000 Y-650.000" in m for m in moves)
    assert not any("Z" in m for m in moves)


def test_pad_hit_maps_each_quadrant():
    # Canvas y grows downward, so "up the screen" is a negative dy.
    r = (PAD_DEAD + PAD_RADII[-1]) / 2  # somewhere in the middle of the pad
    assert pad_hit(r, 0.0)[:2] == ("X", +1)  # right
    assert pad_hit(-r, 0.0)[:2] == ("X", -1)  # left
    assert pad_hit(0.0, -r)[:2] == ("Y", +1)  # up the screen = back
    assert pad_hit(0.0, r)[:2] == ("Y", -1)  # down the screen = front


def test_pad_hit_picks_the_ring_by_radius():
    # Derived from the geometry constants so resizing the pad can't break this.
    prev = PAD_DEAD
    for expected, edge in enumerate(PAD_RADII, start=1):
        midpoint = (prev + edge) / 2
        assert pad_hit(midpoint, 0.0)[2] == expected, (midpoint, expected)
        assert pad_hit(edge, 0.0)[2] == expected  # inclusive at the outer edge
        prev = edge


def test_pad_hit_ignores_dead_centre_and_outside():
    assert pad_hit(0.0, 0.0) is None
    assert pad_hit(PAD_DEAD - 1, 0.0) is None  # inside the dead zone
    assert pad_hit(PAD_RADII[-1] + 1, 0.0) is None  # beyond the outer ring


def test_pad_hit_diagonals_are_stable_at_the_boundaries():
    # Exactly on a 45-degree boundary must still resolve to one quadrant,
    # never to None, so a click can't silently do nothing.
    for deg in range(0, 360, 5):
        rad = math.radians(deg)
        r = (PAD_DEAD + PAD_RADII[-1]) / 2
        hit = pad_hit(r * math.cos(rad), -r * math.sin(rad))
        assert hit is not None, deg
        assert hit[0] in ("X", "Y") and hit[1] in (1, -1)


REPLY = [
    b"[LIM:X3.00..94.00,Y3.00..139.00]\n",
    b"[FEED:MAX2400,TRAVEL1800,MIN30]\n",
    b"[ACC:X800,Y800,V2.00]\n",
    b"ok\n",
]


def connected(fake_serial):
    """A controller on a fake port, past the status-and-limits handshake."""
    ctrl = PicoController("/dev/fake")
    ctrl.connect()
    return ctrl


def words(move):
    """The G-code words of one move, as {letter: value}."""
    return {w[0]: w[1:] for w in move.split()[1:]}


def test_read_envelope_flips_the_sign_of_the_boards_limits(fake_serial):
    # The firmware counts positive millimetres from home; everything above
    # the transport works in the negative frame. The near limit on the board
    # is therefore the FAR limit here, which is the part a naive negation
    # would get backwards.
    ctrl = connected(fake_serial)
    ctrl._serial._queue.extend(REPLY)
    env = read_envelope(ctrl)
    assert (env.lo_x, env.hi_x) == (-94.0, -3.0)
    assert (env.lo_y, env.hi_y) == (-139.0, -3.0)


def test_read_envelope_takes_the_acceleration_too(fake_serial):
    # The lead distances a gradient needs must match what the firmware will
    # actually do. Two copies of that constant would drift apart and the
    # coating would quietly stop matching the recipe.
    ctrl = connected(fake_serial)
    ctrl._serial._queue.extend(REPLY)
    env = read_envelope(ctrl)
    assert env.accel("X") == 800.0
    assert env.accel("Y") == 800.0
    assert env.junction == 2.0


def test_read_envelope_clamps_the_operator_ceiling_to_the_machines(fake_serial):
    # The operator setting is a guard rail below the machine's limit. A
    # value saved against the Shapeoko's 6000 must not survive as licence to
    # ask this stage for a feed it will refuse.
    ctrl = connected(fake_serial)
    ctrl._serial._queue.extend(REPLY)
    assert read_envelope(ctrl, max_feed_xy=6000.0).max_feed_xy == 2400.0
    ctrl._serial._queue.extend(REPLY)
    assert read_envelope(ctrl, max_feed_xy=1500.0).max_feed_xy == 1500.0


def test_read_envelope_falls_back_when_board_is_silent(fake_serial):
    ctrl = connected(fake_serial)

    def boom(*_args, **_kwargs):
        raise TimeoutError("no response")

    ctrl.read_limits = boom
    env = read_envelope(ctrl)
    assert (env.lo_x, env.hi_x, env.lo_y, env.hi_y) == (
        Envelope().lo_x, Envelope().hi_x, Envelope().lo_y, Envelope().hi_y,
    )


# --- line kinds -----------------------------------------------------------


def test_kinds_are_the_three_offered():
    assert set(KINDS) == {"line", "line_back", "dot"}


def test_line_back_returns_at_the_spray_feed():
    ln = SprayLine("line_back", "X", -400, -400, 40, 500, 2, 3.0)
    moves = gcode_only(plan_moves([ln], 2000))
    # Out and back, both at F500; only the initial travel uses F2000.
    assert moves.count("G1 X-360.000 F500") == 2
    assert moves.count("G1 X-400.000 F500") == 2
    assert not [m for m in moves if m == "G1 X-400.000 F2000"]
    assert sum("F2000" in m for m in moves) == 1


def test_plain_line_returns_at_the_travel_feed():
    ln = SprayLine("line", "X", -400, -400, 40, 500, 2, 3.0)
    moves = gcode_only(plan_moves([ln], 2000))
    assert moves.count("G1 X-360.000 F500") == 2
    assert "G1 X-400.000 F2000" in moves  # fast return between passes


def test_dot_travels_once_and_stays_once():
    # Reps and pause are meaningless for a dot: one travel, one stay, move on.
    ln = SprayLine("dot", "X", -222, -333, 0, 500, 3, 2.0, dwell_s=1.5)
    plan = plan_moves([ln], 2000)
    assert gcode_only(plan) == ["G21", "G90", "M5", "G1 X-222.000 Y-333.000 F2000"]
    assert [v for k, v in plan if k == "dwell"] == ["1.5"]
    assert [v for k, v in plan if k == "pause"] == []


def test_dot_keeps_its_trailing_dwell():
    # The trailing *pause* is dropped, but a dot's stay is the spraying itself.
    ln = SprayLine("dot", "X", -222, -333, 0, 500, 1, 0.0, dwell_s=1.0)
    assert plan_moves([ln], 2000)[-1] == ("dwell", "1")


def test_dot_between_lines_does_not_stall_the_run():
    # A dot in the middle stays once, then the next entry travels straight in.
    lines = [
        SprayLine("dot", "X", -222, -333, 0, 500, 1, 0.0, dwell_s=2.0),
        SprayLine("line", "X", -400, -400, 40, 800, 1, 0.0),
    ]
    kinds = [k for k, _ in plan_moves(lines, 2000) if k in ("dwell", "pause", "gcode")]
    assert kinds.count("dwell") == 1
    assert kinds.count("pause") == 0


def test_dot_ignores_the_fields_it_does_not_use():
    # No length, end point, feed, reps or pause is checked for a dot.
    assert SprayLine("dot", "X", -400, -400, 0, 99999, 0, -5.0, dwell_s=2.0).validate(ENV) == []


def test_dot_requires_a_positive_dwell():
    errs = SprayLine("dot", "X", -400, -400, 0, 500, 1, 0.0, dwell_s=0.0).validate(ENV)
    assert any("dwell" in e for e in errs)


def test_dot_start_is_still_bounds_checked():
    errs = SprayLine("dot", "X", -900, -400, 0, 500, 1, 0.0, dwell_s=2.0).validate(ENV)
    assert any("start X" in e for e in errs)


def test_unknown_kind_is_rejected():
    assert any("type" in e for e in SprayLine("squiggle", "X", -400, -400, 10).validate(ENV))


def test_end_xy_of_a_dot_is_its_start():
    assert SprayLine("dot", "X", -111, -222, 999).end_xy() == (-111, -222)


# --- velocity gradient ---------------------------------------------------


def test_gradient_only_applies_to_line_back():
    assert not SprayLine("line", "X", -400, -400, 20, 500, feed_end=2000).has_gradient
    assert SprayLine("line_back", "X", -400, -400, 20, 500, feed_end=2000).has_gradient
    # Equal or zero end feed means a constant-feed pass.
    assert not SprayLine("line_back", "X", -400, -400, 20, 500, feed_end=500).has_gradient
    assert not SprayLine("line_back", "X", -400, -400, 20, 500, feed_end=0).has_gradient


def test_gradient_ramps_linearly_between_the_two_feeds():
    segs = gradient_segments(0.0, 10.0, 500.0, 1500.0, 10)
    feeds = [f for _, f in segs]
    assert feeds[0] == 550.0  # midpoint of the first tenth
    assert feeds[-1] == 1450.0
    steps = {round(b - a, 6) for a, b in zip(feeds, feeds[1:])}
    assert steps == {100.0}  # evenly spaced, i.e. linear


def test_gradient_segments_span_the_whole_line():
    segs = gradient_segments(-400.0, -380.0, 500.0, 1500.0, 10)
    assert segs[-1][0] == -380.0  # last segment lands exactly on the end


def test_gradient_return_mirrors_the_outward_speeds():
    # Every point must be traversed at the same speed in both directions,
    # otherwise the ramp smears instead of shaping the coating.
    out = gradient_segments(-400.0, -380.0, 500.0, 1500.0, 10)
    back = gradient_segments(-380.0, -400.0, 1500.0, 500.0, 10)
    assert [f for _, f in out] == [f for _, f in reversed(back)]


def test_gradient_pass_is_one_move_per_stroke():
    # GRBL held one feed per move, so a ramp had to be chopped into short
    # constant-feed segments. The stage sweeps its step rate continuously,
    # so each stroke is a single command carrying both feeds.
    ln = SprayLine("line_back", "X", -400, -400, 20, 500, 1, 0.0, feed_end=2000)
    ramped = [m for m in gcode_only(plan_moves([ln], 2000, ENV)) if "E" in m]
    assert len(ramped) == 2
    assert "F500" in ramped[0] and "E2000" in ramped[0]
    assert "F2000" in ramped[1] and "E500" in ramped[1]


def test_gradient_return_stroke_reverses_the_feeds():
    # Every point on the line is then traversed at the same speed in both
    # directions, so the gradient shapes the coating instead of smearing it.
    ln = SprayLine("line_back", "Y", -400, -400, 20, 500, 1, 0.0, feed_end=2000)
    ramped = [m for m in gcode_only(plan_moves([ln], 2000, ENV)) if "E" in m]
    out, back = words(ramped[0]), words(ramped[1])
    assert float(out["F"]) == float(back["E"])
    assert float(out["E"]) == float(back["F"])
    assert out["I"] == back["I"] and out["K"] == back["K"]


def test_gradient_runs_in_before_the_span_and_out_past_it():
    # A pass that starts and stops on the coated span cannot be at the
    # requested feed at its own ends, whatever the acceleration. So the
    # ramps are put outside, and the gradient is defined only over what
    # lies between the two lead distances.
    ln = SprayLine("line_back", "Y", -400, -400, 20, 500, 1, 0.0, feed_end=2000)
    lead = lead_for(2000)
    assert lead > 0
    ramped = [m for m in gcode_only(plan_moves([ln], 2000, ENV)) if "E" in m]
    assert f"I{lead:.2f}" in ramped[0] and f"K{lead:.2f}" in ramped[0]
    # Outward stroke overshoots the far end by exactly the lead-out.
    assert f"Y{-380.0 + lead:.3f}" in ramped[0]
    # Return stroke comes back to the lead-in point, short of the start.
    assert f"Y{-400.0 - lead:.3f}" in ramped[1]


def test_gradient_mode_rides_along_as_a_q_word():
    for mode, q in (("velocity", "Q0"), ("thickness", "Q1")):
        ln = SprayLine("line_back", "Y", -400, -400, 20, 500, 1, 0.0,
                       feed_end=2000, gradient_mode=mode)
        ramped = [m for m in gcode_only(plan_moves([ln], 2000, ENV)) if "E" in m]
        assert all(q in m for m in ramped)


def test_gradient_segment_helpers_survive_for_previewing():
    # No longer used to emit G-code, but still the clearest way to sample a
    # profile when checking one by eye.
    assert gradient_count(20) == 10
    assert gradient_count(1) == 2  # never fewer than two
    assert gradient_count(10_000) == MAX_GRADIENT_SEGMENTS


def test_end_feed_is_range_checked():
    env = Envelope(min_x=-845.0, max_x=-6.0, min_y=-850.0, max_y=-6.0,
                   max_feed_xy=4000.0)
    assert SprayLine("line_back", "X", -400, -400, 20, 500, feed_end=4000).validate(env) == []
    errs = SprayLine("line_back", "X", -400, -400, 20, 500, feed_end=4001).validate(env)
    assert any("end feed" in e for e in errs)


def test_end_feed_on_the_wrong_type_is_rejected():
    errs = SprayLine("line", "X", -400, -400, 20, 500, feed_end=2000).validate(ENV)
    assert any("Line/back" in e for e in errs)


# --- thickness gradient mode ---------------------------------------------


def test_both_gradient_modes_hit_the_same_endpoints():
    # The modes differ in the curve between the ends, never at the ends.
    for mode in GRADIENT_MODES:
        assert gradient_feed(0.0, 500.0, 2000.0, mode) == pytest.approx(500.0)
        assert gradient_feed(1.0, 500.0, 2000.0, mode) == pytest.approx(2000.0)


def test_velocity_mode_is_linear_in_speed():
    feeds = [gradient_feed(i / 10, 500.0, 1500.0, "velocity") for i in range(11)]
    steps = {round(b - a, 6) for a, b in zip(feeds, feeds[1:])}
    assert steps == {100.0}


def test_thickness_mode_is_linear_in_deposited_coating():
    # Coating per unit length goes as 1/v, so 1/feed must be evenly spaced.
    feeds = [gradient_feed(i / 10, 500.0, 1500.0, "thickness") for i in range(11)]
    deposition = [1.0 / f for f in feeds]
    steps = [round(b - a, 12) for a, b in zip(deposition, deposition[1:])]
    assert steps == pytest.approx([steps[0]] * len(steps))


def test_the_two_modes_actually_differ_in_between():
    v = gradient_feed(0.5, 500.0, 2000.0, "velocity")
    t = gradient_feed(0.5, 500.0, 2000.0, "thickness")
    assert v == pytest.approx(1250.0)
    assert t == pytest.approx(800.0)  # harmonic mean, not arithmetic
    assert t < v


def test_thickness_mode_return_stroke_still_mirrors():
    # The mirror identity has to hold for both modes or the ramp smears.
    out = gradient_segments(-400.0, -380.0, 500.0, 2000.0, 10, "thickness")
    back = gradient_segments(-380.0, -400.0, 2000.0, 500.0, 10, "thickness")
    assert [f for _, f in out] == pytest.approx([f for _, f in reversed(back)])


def test_planner_uses_the_selected_mode():
    common = dict(kind="line_back", axis="X", start_x=-400, start_y=-400, length=20, feed=500)
    vel = SprayLine(**common, feed_end=2000, gradient_mode="velocity")
    thk = SprayLine(**common, feed_end=2000, gradient_mode="thickness")
    fv = [m for m in gcode_only(plan_moves([vel], 2000)) if "F" in m][1:]
    ft = [m for m in gcode_only(plan_moves([thk], 2000)) if "F" in m][1:]
    assert len(fv) == len(ft)
    assert fv != ft


def test_unknown_gradient_mode_is_rejected():
    ln = SprayLine("line_back", "X", -400, -400, 20, 500, feed_end=2000, gradient_mode="wobble")
    assert any("gradient mode" in e for e in ln.validate(ENV))


# --- continuous motion ----------------------------------------------------


def test_consecutive_moves_do_not_wait_for_the_machine_to_stop():
    # Stopping between segments is what makes a feed ramp lurch.
    ln = SprayLine("line_back", "X", -400, -400, 20, 500, 1, 0.0, feed_end=2000)
    plan = plan_moves([ln], 2000)
    waits = [_next_needs_stillness(plan, i) for i, (k, _) in enumerate(plan) if k == "gcode"]
    assert waits[-1] is True  # the last move must settle
    assert not any(waits[:-1])  # everything before it flows straight on


def test_the_machine_settles_before_a_drying_pause():
    # A pause has to happen at the end of the stroke, not mid-flight.
    ln = SprayLine("line", "X", -400, -400, 40, 800, 2, 3.0)
    plan = plan_moves([ln], 2000)
    for i, (kind, _) in enumerate(plan):
        if kind == "gcode" and i + 1 < len(plan) and plan[i + 1][0] == "pause":
            assert _next_needs_stillness(plan, i) is True


def test_the_machine_settles_before_a_dot_dwell():
    ln = SprayLine("dot", "X", -222, -333, 0, 500, 1, 0.0, dwell_s=2.0)
    plan = plan_moves([ln], 2000)
    travel = max(i for i, (k, _) in enumerate(plan) if k == "gcode")
    assert _next_needs_stillness(plan, travel) is True


def test_notes_between_moves_do_not_force_a_stop():
    # Progress notes sit between passes; they must not break the motion.
    ln = SprayLine("line", "X", -400, -400, 40, 800, 3, 0.0)
    plan = plan_moves([ln], 2000)
    assert any(k == "note" for k, _ in plan)
    gcode_idx = [i for i, (k, _) in enumerate(plan) if k == "gcode"]
    assert not any(_next_needs_stillness(plan, i) for i in gcode_idx[:-1])


# --- speed scale ----------------------------------------------------------


def test_scale_lines_is_a_no_op_at_one():
    lines = [SprayLine("line", "X", -400, -400, 40, 800, 2, 3.0)]
    assert scale_lines(lines, 1.0, 6000.0) == lines


def test_scale_lines_multiplies_every_spray_feed():
    lines = [SprayLine("line_back", "X", -400, -400, 40, 800, 2, 3.0, feed_end=1600)]
    out = scale_lines(lines, 2.0, 6000.0)
    assert (out[0].feed, out[0].feed_end) == (1600.0, 3200.0)


def test_scale_lines_clamps_at_the_ceiling():
    lines = [SprayLine("line_back", "X", -400, -400, 40, 4000, 1, 0.0, feed_end=5000)]
    out = scale_lines(lines, 4.0, 6000.0)
    assert (out[0].feed, out[0].feed_end) == (6000.0, 6000.0)


def test_scale_lines_leaves_durations_alone():
    # Pause and dwell are times, not speeds.
    lines = [SprayLine("dot", "X", -222, -333, 0, 500, 1, 4.0, dwell_s=2.5)]
    out = scale_lines(lines, 0.5, 6000.0)
    assert (out[0].pause_s, out[0].dwell_s) == (4.0, 2.5)


def test_scale_lines_does_not_mutate_the_original():
    lines = [SprayLine("line", "X", -400, -400, 40, 800, 1, 0.0)]
    scale_lines(lines, 3.0, 6000.0)
    assert lines[0].feed == 800.0


def test_scaled_gradient_keeps_its_shape():
    # Halving the speed should halve every feed in the ramp, not distort it.
    ln = SprayLine("line_back", "X", -400, -400, 20, 1000, 1, 0.0, feed_end=2000)
    full = [f for _, f in gradient_segments(0, 20, ln.feed, ln.feed_end, 10)]
    half_ln = scale_lines([ln], 0.5, 6000.0)[0]
    half = [f for _, f in gradient_segments(0, 20, half_ln.feed, half_ln.feed_end, 10)]
    assert [round(f / 2, 6) for f in full] == [round(f, 6) for f in half]


# --- control script -------------------------------------------------------


def test_control_script_plans_independently_of_the_main_recipe():
    # Ctrl RUN must plan from its own lines, never the table's.
    main = [SprayLine("dot", "X", -200, -200, 0, 500, 1, 0.0, dwell_s=3.0)]
    control = [SprayLine("line_back", "Y", -130, -250, 30, 800, 2, 0.0)]
    assert gcode_only(plan_moves(main, 2000)) != gcode_only(plan_moves(control, 2000))
    assert any("Y-220.000" in m for m in gcode_only(plan_moves(control, 2000)))


def test_control_script_round_trips_through_json():
    lines = [
        SprayLine("line_back", "Y", -130, -250, 30, 800, 4, 0.0, feed_end=1600),
        SprayLine("dot", "X", -222, -333, 0, 500, 1, 0.0, dwell_s=2.0),
    ]
    blob = json.dumps({"lines": [asdict(x) for x in lines]})
    assert [SprayLine(**d) for d in json.loads(blob)["lines"]] == lines


def test_control_script_obeys_the_speed_scale():
    control = [SprayLine("line", "Y", -130, -250, 30, 1000, 1, 0.0)]
    assert scale_lines(control, 0.5, 6000.0)[0].feed == 500.0


# --- script mode ----------------------------------------------------------

FIVE = [SprayLine("line", "Y", -130, -300, 20, 800, 1, 0.0) for _ in range(5)]


def test_parses_the_sonotek_style_example():
    stmts, errors = parse_script("LOOP(1,3,5) = 3; TRANSLATE (10;20); WAIT=3; LOOP (1,3,5) =3")
    assert errors == []
    assert [s.op for s in stmts] == ["LOOP", "TRANSLATE", "WAIT", "LOOP"]
    assert stmts[0].lines == (1, 3, 5) and stmts[0].count == 3
    assert (stmts[1].dx, stmts[1].dy) == (10.0, 20.0)
    assert stmts[2].seconds == 3.0


def test_semicolon_inside_translate_is_not_a_statement_break():
    # ';' separates statements *and* TRANSLATE's arguments; splitting naively
    # would tear the statement in half.
    stmts, errors = parse_script("TRANSLATE(10;20); WAIT=1")
    assert errors == []
    assert [s.op for s in stmts] == ["TRANSLATE", "WAIT"]


def test_commas_and_semicolons_both_work_inside_translate():
    a, _ = parse_script("TRANSLATE(10;20)")
    b, _ = parse_script("TRANSLATE(10,20)")
    assert (a[0].dx, a[0].dy) == (b[0].dx, b[0].dy) == (10.0, 20.0)


def test_loop_list_is_a_list_not_a_range():
    assert parse_script("LOOP(1,3,5)=1")[0][0].lines == (1, 3, 5)
    assert parse_script("LOOP(1,3-5)=1")[0][0].lines == (1, 3, 4, 5)


def test_loop_preserves_the_order_written():
    assert parse_script("LOOP(5,1,3)=1")[0][0].lines == (5, 1, 3)


def test_script_rejects_malformed_statements():
    for bad, hint in [
        ("LOOP(1,3,5)", "expected LOOP"),
        ("LOOP(0)=1", "numbered from 1"),
        ("LOOP(5-3)=1", "backwards"),
        ("TRANSLATE(10)", "two numbers"),
        ("WAIT", "expected WAIT"),
        ("SPIN(3)", "unknown command"),
    ]:
        errors = parse_script(bad)[1]
        assert errors and hint in errors[0], (bad, errors)


def test_comments_and_blank_statements_are_ignored():
    stmts, errors = parse_script("# a comment\nLOOP(1)=1;; # trailing\n")
    assert errors == []
    assert [s.op for s in stmts] == ["LOOP"]


def test_translate_accumulates_and_origin_resets():
    stmts, _ = parse_script(
        "LOOP(1)=1; TRANSLATE(10;0); LOOP(1)=1; TRANSLATE(10;0); LOOP(1)=1; ORIGIN; LOOP(1)=1"
    )
    items, errors = expand_script(FIVE, stmts)
    assert errors == []
    assert [round(ln.start_x, 3) for _, ln, _ in items] == [-130.0, -120.0, -110.0, -130.0]


def test_loop_runs_the_group_in_order_for_each_repetition():
    stmts, _ = parse_script("LOOP(1,3)=2")
    ctx = [where for _, _, where in expand_script(FIVE, stmts)[0]]
    assert "loop 1/2 entry 1" in ctx[0]
    assert "loop 1/2 entry 3" in ctx[1]
    assert "loop 2/2 entry 1" in ctx[2]


def test_a_loop_respects_each_entrys_own_repetitions():
    # LOOP(3)=3 on an entry with Reps=2 executes the entry fully each time.
    lines = [SprayLine("line", "Y", -130, -300, 20, 800, 2, 0.0)]
    stmts, _ = parse_script("LOOP(1)=3")
    plan = plan_script(lines, stmts, 2000)
    # Count real spray moves, not log text: the header note says "2 pass(es)".
    assert sum(1 for k, v in plan if k == "gcode" and "F800" in v) == 6


def test_missing_entry_is_reported_with_its_statement():
    errors = expand_script(FIVE, parse_script("LOOP(9)=1")[0])[1]
    assert errors and "entry 9 does not exist" in errors[0]


def test_translated_entries_are_bounds_checked_not_the_table():
    # Every entry sits inside the envelope; only the translated copy does not.
    assert all(ln.validate(ENV) == [] for ln in FIVE)
    stmts, _ = parse_script("LOOP(1)=1; TRANSLATE(0;-700); LOOP(1)=1")
    items, _ = expand_script(FIVE, stmts)
    errors = validate_script_items(items, ENV)
    assert errors
    assert "offset +0,-700" in errors[0]  # names the offending repetition


def test_script_plan_has_the_same_shape_as_a_sequential_one():
    stmts, _ = parse_script("LOOP(1)=1")
    assert gcode_only(plan_script(FIVE, stmts, 2000)) == gcode_only(plan_moves(FIVE[:1], 2000))


def test_wait_becomes_a_host_side_pause():
    stmts, _ = parse_script("LOOP(1)=1; WAIT=2.5; LOOP(1)=1")
    assert [v for k, v in plan_script(FIVE, stmts, 2000) if k == "pause"] == ["2.5"]


def test_a_script_ending_on_wait_drops_it():
    # Same rule as sequential mode: nothing to wait for before the cup.
    stmts, _ = parse_script("LOOP(1)=1; WAIT=3")
    assert plan_script(FIVE, stmts, 2000)[-1][0] != "pause"


# --- run time estimate ----------------------------------------------------


def test_estimate_counts_travel_and_waits():
    # One 20 mm pass at F600 is 2 s; the wait adds 3 s on top.
    lines = [SprayLine("line", "Y", -130, -300, 20, 600, 1, 0.0)]
    plan = plan_moves(lines, 2000)
    assert estimate_seconds(plan, (-130, -300)) == pytest.approx(2.0, abs=0.05)
    # 2 s pass + 3 s wait + 0.6 s travel back to the start + 2 s second pass.
    stmts, _ = parse_script("LOOP(1)=1; WAIT=3; LOOP(1)=1")
    assert estimate_seconds(plan_script(lines, stmts, 2000), (-130, -300)) == pytest.approx(
        7.6, abs=0.1
    )


def test_estimate_scales_with_feed():
    lines = [SprayLine("line", "Y", -130, -300, 20, 600, 1, 0.0)]
    slow = estimate_seconds(plan_moves(lines, 2000), (-130, -300))
    fast = estimate_seconds(plan_moves(scale_lines(lines, 2.0, 6000), 2000), (-130, -300))
    assert fast == pytest.approx(slow / 2, abs=0.05)


def test_estimate_skips_the_opening_move_without_a_start():
    # Where the head begins is not known at plan time, so that leg is excluded.
    lines = [SprayLine("line", "Y", -130, -300, 20, 600, 1, 0.0)]
    plan = plan_moves(lines, 2000)
    assert estimate_seconds(plan) < estimate_seconds(plan, (-500, -500))


def test_duration_formats_readably():
    assert format_duration(9.4) == "9s"
    assert format_duration(65) == "1m05s"
    assert format_duration(3599) == "59m59s"


def test_cup_move_is_always_a_single_move():
    # One move, so there is no second leg for the run to wait on. This used
    # to matter a great deal: a zero-length Z leg finished before a status
    # poll could catch the machine moving, and every pattern ended with a
    # visible stall while the wait fell through its grace period.
    for cur_z in (-70.02, -70.0, -30.0):
        assert len(plan_cup_move(cur_z, (-130.0, -180.0, -70.02), 2000)) == 1


# --- script insert menu ---------------------------------------------------


def test_every_menu_snippet_parses_on_its_own():
    # An inserted command must be valid immediately, not a placeholder that
    # errors until it is filled in.
    for _label, snippet in SCRIPT_SNIPPETS:
        stmts, errors = parse_script(snippet)
        assert errors == [], (snippet, errors)
        assert len(stmts) == 1


def test_menu_snippets_cover_every_command():
    inserted = {parse_script(snip)[0][0].op for _, snip in SCRIPT_SNIPPETS}
    assert inserted == {"LOOP", "TRANSLATE", "ORIGIN", "WAIT"}


def test_snippet_selection_targets_the_editable_part():
    assert "LOOP(1,3,5)=3"[slice(*snippet_selection("LOOP(1,3,5)=3"))] == "1,3,5"
    assert "TRANSLATE(10;20)"[slice(*snippet_selection("TRANSLATE(10;20)"))] == "10;20"
    assert "WAIT=3"[slice(*snippet_selection("WAIT=3"))] == "3"
    assert snippet_selection("ORIGIN") is None


def test_all_snippets_together_form_a_valid_script():
    joined = "\n".join(snip for _, snip in SCRIPT_SNIPPETS)
    assert parse_script(joined)[1] == []


# --- the transport ---------------------------------------------------------
#
# PicoController speaks GRBL upward and the stage's dialect downward. Two
# bugs lived here and both were invisible from the GUI: a modal G90 prefix
# that stopped a move reaching the Z-stripper, and a connect that waited for
# a banner the board had already printed. The translation is pure, so it can
# be tested without a port at all.


def shim(pos=(-50.0, -50.0)):
    ctrl = PicoController.__new__(PicoController)
    ctrl._last_feed = 0.0
    ctrl._pos = pos
    ctrl.limits = Limits()
    return ctrl


def test_limits_are_negative_with_the_far_end_first():
    lim = Limits()
    assert lim.min_x < lim.max_x < 0
    assert lim.min_y < lim.max_y < 0


def test_coordinates_are_negated_on_the_way_to_the_board():
    assert shim()._translate("G1 X-20.000 Y-30.000 F1800") == "G1 X20.000 Y30.000 F1800"


def test_status_is_negated_on_the_way_back():
    ctrl = shim()
    st = ctrl._note_status("<Idle|MPos:86.580,9.930>")
    assert st.mpos == (-86.58, -9.93, 0.0)
    assert ctrl._pos == (-86.58, -9.93)


def test_an_unhomed_board_reports_no_position():
    st = shim()._note_status("<Alarm|MPos:none>")
    assert st.state == "Alarm"
    assert st.mpos is None


def test_a_modal_g90_still_reaches_the_move_translator():
    # The bug: dispatching on a startswith("G1") test missed this line
    # entirely, so Z and a zero feed went straight to the firmware.
    assert shim()._translate("G90 G1 Z-70.050 F0") is None
    assert shim()._translate("G90 G1 X-20.000 Y-30.000 F1800") == "G1 X20.000 Y30.000 F1800"


def test_z_is_dropped_but_xy_survives_beside_it():
    assert shim()._translate("G1 X-20.000 Y-30.000 Z-70.000 F900") == "G1 X20.000 Y30.000 F900"


def test_the_preamble_is_swallowed():
    for noop in ("G21", "G90", "M5", "G21 G90"):
        assert shim()._translate(noop) is None


def test_gradient_words_ride_through_untouched():
    out = shim()._translate("G1 Y-61.751 F600 E2400 I1.50 K1.50 Q1")
    assert out == "G1 Y61.751 F600 E2400 I1.50 K1.50 Q1"


def test_a_relative_jog_becomes_an_absolute_move():
    # The firmware has no jog and no relative mode, so the transport has to
    # know where the machine is to turn one into the other.
    assert shim()._translate("$J=G91 G21 X10.000 F600") == "G1 X40.000 Y50.000 F600"
    assert shim()._translate("$J=G91 G21 X-10.000 F600") == "G1 X60.000 Y50.000 F600"


def test_a_jog_before_homing_is_refused_rather_than_guessed():
    ctrl = shim(pos=None)
    with pytest.raises(Exception):
        ctrl._translate("$J=G91 G21 X10.000 F600")


def test_system_commands_pass_straight_through():
    for cmd in ("$H", "$$", "?", "!"):
        assert shim()._translate(cmd) == cmd


def test_lead_grows_with_the_square_of_the_feed():
    # v^2 - v0^2 = 2*a*d, so doubling the feed roughly quadruples the
    # run-up. This is why the fast end of a gradient is the expensive one.
    # Only roughly: the junction speed the move starts from is subtracted,
    # and at low feeds that term is a real fraction of the answer.
    lim = Limits()
    near, far = lim.lead(1200, "Y") - 0.5, lim.lead(2400, "Y") - 0.5
    assert far == pytest.approx(4 * near, rel=0.02)
    # At a low feed the junction speed shows. Subtracting v0^2 takes a
    # larger fraction of a small number, so halving the feed gives slightly
    # LESS than a quarter of the run-up, not slightly more.
    assert lim.lead(600, "Y") - 0.5 < (lim.lead(1200, "Y") - 0.5) / 4


def test_lead_matches_what_the_envelope_computes():
    # One source of truth: the host must not ask for a lead the firmware
    # will not honour.
    assert Limits().lead(2400, "X") == pytest.approx(Envelope().lead(2400, "X"))


# --- time estimates for ramped moves --------------------------------------


def test_a_ramped_move_is_not_distance_over_mean_feed():
    # Time is the integral of 1/v, and the slow end dominates it. Using the
    # mean feed underestimates an 18 mm 600->2400 pass by about a third.
    ln = SprayLine("line_back", "Y", -400, -400, 20, 600, 1, 0.0,
                   feed_end=2400, gradient_mode="thickness")
    ramped = estimate_seconds(plan_moves([ln], 1800, ENV), start=(-400.0, -400.0))
    flat = SprayLine("line_back", "Y", -400, -400, 20, 1500, 1, 0.0)
    naive = estimate_seconds(plan_moves([flat], 1800, ENV), start=(-400.0, -400.0))
    assert ramped > naive


def test_thickness_mode_takes_longer_than_velocity_mode():
    # Thickness mode holds the slow end longer, because it is 1/v that is
    # interpolated linearly rather than v.
    def secs(mode):
        ln = SprayLine("line_back", "Y", -400, -400, 20, 600, 1, 0.0,
                       feed_end=2400, gradient_mode=mode)
        return estimate_seconds(plan_moves([ln], 1800, ENV), start=(-400.0, -400.0))

    assert secs("thickness") > secs("velocity")


# --- per-move ack timeouts -------------------------------------------------
#
# The firmware acks a move when it has FINISHED, so these are what stands
# between a board that stops answering and a run that cannot be stopped.


def _firmware_bound(length: float, f0: float, f1: float) -> float:
    """What skr_pico_stage._sweep allows a move before calling it stuck."""
    return 3.0 * length / (min(f0, f1) / 60.0) + 2.0


def test_a_move_timeout_is_longer_than_the_firmwares_own_bound():
    # Ordering that matters: the board must get to report "timed out" for
    # itself. If the host gave up first, every stuck move would look like a
    # dead serial link and the real reason would never reach the log.
    for length, feed in ((76.7, 2400), (50.0, 1000), (20.0, 500), (136.0, 300)):
        plan = [("gcode", f"G1 Y{-length:.3f} F{feed:g}")]
        host = move_timeouts(plan, start=(0.0, 0.0))[0]
        assert host > _firmware_bound(length, feed, feed), (length, feed)


def test_a_move_timeout_still_leaves_room_for_the_move_itself():
    # Four times nominal, so no legitimate move can trip it - but bounded,
    # which is the whole point: the flat 600 s it replaced meant a silent
    # board held the run busy for ten minutes.
    plan = [("gcode", "G1 Y-76.714 F2400")]
    (wait,) = move_timeouts(plan, start=(0.0, 0.0))
    nominal = 76.714 / (2400 / 60.0)
    assert wait > 4 * nominal
    assert wait < 600.0


def test_a_gradient_gets_a_longer_timeout_than_its_mean_feed_implies():
    # _move_seconds integrates 1/v, so a ramped pass is allowed the time it
    # actually takes rather than the time its average suggests. Long spans,
    # because every pass in a real recipe is short enough to sit on the floor
    # and the floor would hide the difference.
    ramped = move_timeouts([("gcode", "G1 Y-120 F500 E1800 I1.06 K1.06 Q1")],
                           start=(0.0, 0.0))[0]
    flat = move_timeouts([("gcode", "G1 Y-120 F1150")], start=(0.0, 0.0))[0]
    assert ramped > flat > 15.0


def test_setup_lines_and_unmeasurable_moves_fall_back_to_the_floor():
    plan = [("gcode", "G21"), ("gcode", "G90"), ("gcode", "M5"),
            ("note", "1: Line X 50 mm"), ("pause", "0")]
    assert move_timeouts(plan, start=(0.0, 0.0)) == [15.0] * len(plan)
    # No starting position: the opening move cannot be measured, so it takes
    # the floor rather than a guessed distance - and the NEXT one, which can
    # be measured from it, does not.
    waits = move_timeouts([("gcode", "G1 X-50.000 Y-50.000 F2400"),
                           ("gcode", "G1 Y-120.000 F600")], start=None)
    assert waits[0] == 15.0
    assert waits[1] > 15.0


# --- port discovery across platforms --------------------------------------
#
# Name matching alone found nothing on Windows, where every port is COMn, so
# the chooser came up empty and there was nothing to connect to.


class _FakePort:
    def __init__(self, device, description="", manufacturer=None, vid=None):
        self.device = device
        self.description = description
        self.manufacturer = manufacturer
        self.vid = vid


def _ports(monkeypatch, *fakes):
    from spray_stage import pico_controller as pc

    monkeypatch.setattr(pc.list_ports, "comports", lambda: list(fakes))
    return [p.device for p in pc.find_ports()]


def test_a_windows_com_port_is_found_by_vendor_id(monkeypatch):
    from spray_stage.pico_controller import RP2040_VID

    found = _ports(
        monkeypatch,
        _FakePort("COM3", "Standard Serial over Bluetooth link"),
        _FakePort("COM5", "USB Serial Device", vid=RP2040_VID),
    )
    assert found[0] == "COM5"


def test_the_mac_device_name_still_wins_over_an_unmarked_port(monkeypatch):
    found = _ports(
        monkeypatch,
        _FakePort("/dev/cu.Bluetooth-Incoming-Port", ""),
        _FakePort("/dev/cu.usbmodem101", "Board in FS mode"),
    )
    assert found[0] == "/dev/cu.usbmodem101"


def test_a_vendor_id_match_outranks_a_mere_name_match(monkeypatch):
    from spray_stage.pico_controller import RP2040_VID

    found = _ports(
        monkeypatch,
        _FakePort("/dev/cu.usbserial-1420", "FT232R"),
        _FakePort("/dev/cu.usbmodem101", "Board in FS mode", vid=RP2040_VID),
    )
    assert found == ["/dev/cu.usbmodem101", "/dev/cu.usbserial-1420"]


def test_when_nothing_matches_every_port_is_offered(monkeypatch):
    # A generic CDC driver can leave a Windows port with no identifying marks.
    # An empty chooser is a dead end; a wrong guess is only a wrong guess.
    assert _ports(monkeypatch, _FakePort("COM1"), _FakePort("COM2")) == ["COM1", "COM2"]
