"""Tests for the spray-line planner and validation.

Only the pure logic is covered — instantiating `SprayApp` needs a display,
so the Tk layer is deliberately left untested here.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict

import pytest

from spray_stage.controller import ShapeokoController
from spray_stage.spray_gui import (
    DEFAULT_MAX_FEED_XY,
    DEFAULT_MAX_RATE_XY,
    DEFAULT_MAX_X,
    DEFAULT_MAX_Y,
    DEFAULT_PULLOFF,
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

ENV = Envelope(max_x=845.0, max_y=850.0, max_z=95.0, pulloff=6.0)


def gcode_only(plan):
    return [payload for kind, payload in plan if kind == "gcode"]


def test_zero_is_the_home_corner():
    # Home is back-right with Z up; the bed lies in negative X and Y.
    assert (ENV.hi_x, ENV.hi_y, ENV.hi_z) == (-6.0, -6.0, -6.0)
    assert (ENV.lo_x, ENV.lo_y, ENV.lo_z) == (-845.0, -850.0, -90.0)


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


def test_feed_ceiling_defaults_to_6000_and_is_an_operator_setting():
    assert DEFAULT_MAX_FEED_XY == 6000.0
    assert ENV.max_feed_xy == 6000.0
    assert SprayLine("line", "X", -400, -400, 20, 6000).validate(ENV) == []
    assert any("feed" in e for e in SprayLine("line", "X", -400, -400, 20, 6001).validate(ENV))


def test_feed_ceiling_is_per_envelope_not_global():
    slow = Envelope(max_feed_xy=2000.0)
    fast = Envelope(max_feed_xy=9000.0)
    line = SprayLine("line", "X", -400, -400, 20, 5000)
    assert any("feed" in e for e in line.validate(slow))
    assert line.validate(fast) == []


def test_firmware_rate_is_separate_from_the_operator_ceiling():
    # $110/$111 is the hard bound the operator setting must stay under.
    assert DEFAULT_MAX_RATE_XY == 10000.0
    assert ENV.max_rate_xy == 10000.0


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
    assert ENV.clamp_error_xyz("jog", *jog_target((-10.0, -200.0, -50.0), "X", +1, 10.0))
    assert ENV.clamp_error_xyz("jog", *jog_target((-840.0, -200.0, -50.0), "X", -1, 10.0))
    assert ENV.clamp_error_xyz("jog", *jog_target((-400.0, -200.0, -10.0), "Z", +1, 10.0))
    assert ENV.clamp_error_xyz("jog", *jog_target((-400.0, -200.0, -85.0), "Z", -1, 10.0))
    assert not ENV.clamp_error_xyz("jog", *jog_target((-400.0, -200.0, -50.0), "X", +1, 10.0))
    assert not ENV.clamp_error_xyz("jog", *jog_target((-400.0, -200.0, -50.0), "Z", -1, 10.0))


def test_jog_to_edge_reaches_the_limits():
    # Hold-to-move runs to the edge of the envelope, never past it.
    p = (-400.0, -200.0, -50.0)
    assert jog_to_edge(p, "X", +1, ENV) == 394.0  # -6 - (-400)
    assert jog_to_edge(p, "X", -1, ENV) == -445.0  # -845 - (-400)
    assert jog_to_edge(p, "Y", +1, ENV) == 194.0  # -6 - (-200)
    assert jog_to_edge(p, "Y", -1, ENV) == -650.0  # -850 - (-200)
    assert jog_to_edge(p, "Z", +1, ENV) == 44.0  # -6 - (-50)
    assert jog_to_edge(p, "Z", -1, ENV) == -40.0  # -90 - (-50)


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
    assert jog_to_edge((-400.0, -400.0, -6.0), "Z", +1, ENV) == 0.0
    assert jog_to_edge((-400.0, -400.0, -90.0), "Z", -1, ENV) == 0.0


def test_cup_z_is_bounds_checked():
    assert ENV.clamp_error_z("cup", -55.0) == []
    assert ENV.clamp_error_z("cup", -90.0) == []  # the usable floor
    assert ENV.clamp_error_z("cup", -91.0)  # into the unusable bottom margin
    assert ENV.clamp_error_z("cup", -5.0)  # inside the pull-off, unreachable
    assert ENV.clamp_error_z("cup", 10.0)  # positive is off the machine


def test_z_floor_keeps_clear_of_the_bottom_of_travel():
    # $132 is 95 mm, but the last 5 mm are not usable on this machine.
    assert ENV.max_z == 95.0
    assert ENV.lo_z == -90.0


def test_cup_move_retracts_before_traversing_when_going_up():
    # Cup is higher than the head (less negative Z): lift first, then traverse.
    moves = plan_cup_move(-70.0, (-745.0, -650.0, -30.0), 4000)
    assert moves[0].startswith("G90 G1 Z")
    assert moves[1].startswith("G90 G1 X")


def test_cup_move_traverses_before_descending_when_going_down():
    # Cup is lower: cross at the current height, then come down.
    moves = plan_cup_move(-30.0, (-745.0, -650.0, -70.0), 4000)
    assert moves[0].startswith("G90 G1 X")
    assert moves[1].startswith("G90 G1 Z")


def test_cup_move_never_traverses_below_either_endpoint():
    # Whichever order is chosen, the XY traverse must happen at the higher
    # (less negative) of the two heights.
    for cur_z, cup_z in [(-70.0, -30.0), (-30.0, -70.0), (-40.0, -40.0), (-90.0, -6.0)]:
        moves = plan_cup_move(cur_z, (-745.0, -650.0, cup_z), 4000)
        z_first = moves[0].startswith("G90 G1 Z")
        traverse_z = cup_z if z_first else cur_z
        assert traverse_z == max(cur_z, cup_z)


def test_cup_move_sends_coordinates_untranslated():
    moves = plan_cup_move(-90.0, (-745.0, -650.0, -55.0), 4000)
    assert any("X-745.000 Y-650.000" in m for m in moves)
    assert any("Z-55.000" in m for m in moves)


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


def test_read_envelope_parses_board_settings(fake_serial):
    ctrl = ShapeokoController("/dev/fake")
    ctrl.connect()
    ctrl._serial._queue.extend(
        [b"$27=6.000\n", b"$130=845.000\n", b"$131=850.000\n", b"ok\n"],
    )
    env = read_envelope(ctrl)
    assert (env.pulloff, env.max_x, env.max_y) == (6.0, 845.0, 850.0)


def test_read_envelope_takes_the_slower_of_the_two_axis_rates(fake_serial):
    ctrl = ShapeokoController("/dev/fake")
    ctrl.connect()
    ctrl._serial._queue.extend([b"$110=10000.000\n", b"$111=8000.000\n", b"ok\n"])
    assert read_envelope(ctrl).max_rate_xy == 8000.0


def test_read_envelope_preserves_the_operator_ceiling(fake_serial):
    # The board has no opinion about the operator setting; re-reading it on
    # connect must not silently reset the user's choice.
    ctrl = ShapeokoController("/dev/fake")
    ctrl.connect()
    ctrl._serial._queue.extend([b"$130=845.000\n", b"ok\n"])
    assert read_envelope(ctrl, max_feed_xy=3500.0).max_feed_xy == 3500.0


def test_read_envelope_falls_back_when_board_is_silent(fake_serial):
    ctrl = ShapeokoController("/dev/fake")
    ctrl.connect()

    def boom(*_args, **_kwargs):
        raise TimeoutError("no response")

    ctrl.send = boom
    env = read_envelope(ctrl)
    assert (env.pulloff, env.max_x, env.max_y) == (
        DEFAULT_PULLOFF,
        DEFAULT_MAX_X,
        DEFAULT_MAX_Y,
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


def test_gradient_pass_emits_both_strokes_at_ramped_feeds():
    ln = SprayLine("line_back", "X", -400, -400, 20, 500, 1, 0.0, feed_end=2000)
    moves = [m for m in gcode_only(plan_moves([ln], 2000)) if "F" in m][1:]
    n = gradient_count(20)
    assert len(moves) == 2 * n
    assert moves[0].endswith("F575")  # first outward segment
    assert moves[-1].endswith("F575")  # last return segment, same place


def test_gradient_segment_count_scales_with_length_and_is_capped():
    assert gradient_count(20) == 10
    assert gradient_count(1) == 2  # never fewer than two
    assert gradient_count(10_000) == MAX_GRADIENT_SEGMENTS


def test_end_feed_is_range_checked():
    env = Envelope(max_feed_xy=4000.0)
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


def test_cup_move_skips_a_zero_length_z_leg():
    # Spraying happens at the cup's height, so the Z leg is normally a no-op.
    # Sending it stalls the run: GRBL finishes it before a status poll can see
    # the machine move, so the wait falls through to its grace period.
    moves = plan_cup_move(-70.02, (-130.0, -180.0, -70.02), 2000)
    assert moves == ["G90 G1 X-130.000 Y-180.000 F2000"]


def test_cup_move_still_emits_z_when_it_actually_moves():
    assert len(plan_cup_move(-70.0, (-130.0, -180.0, -30.0), 2000)) == 2
    assert len(plan_cup_move(-30.0, (-130.0, -180.0, -70.0), 2000)) == 2


def test_cup_move_z_threshold_is_tight_enough_to_matter():
    # A real standoff change of a tenth of a millimetre must survive.
    assert len(plan_cup_move(-70.0, (-130.0, -180.0, -70.1), 2000)) == 2


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
