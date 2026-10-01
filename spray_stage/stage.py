"""The machine itself: travel, feeds, acceleration.

Split out of spray_gui so that anything needing to know the shape of the
machine - the recipe migrator, a test, a script - does not have to import
Tkinter to find out. The GUI imports these names back, so there is still
exactly one definition of each.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

# Fallbacks, replaced by the stage's own $$ values once connected.
# Coordinates are POSITIVE here, unlike the Shapeoko: zero is the home
# corner and the numbers grow away from it, which is how the stage firmware
# reports position. Recipes written for the Shapeoko need converting - see
# migrate_recipes.py.
# Negative, with zero at the home corner: MIN is the far end of travel and
# MAX is as close to the switches as the machine parks. The firmware reports
# the mirror image and the transport flips it, so these are what the
# operator sees.
DEFAULT_MIN_X = -94.0
DEFAULT_MAX_X = -3.0
DEFAULT_MIN_Y = -139.0
DEFAULT_MAX_Y = -3.0
DEFAULT_MAX_Z = 0.0
DEFAULT_PULLOFF = 3.0
# The XY feed ceiling is an operator setting, not a firmware one: it is set in
# the GUI and saved. The firmware maximum ($110/$111) is a separate, harder
# limit that this can never exceed.
DEFAULT_MAX_FEED_XY = 2400.0
DEFAULT_MAX_RATE_XY = 2400.0  # measured step-loss cliff
# What travel moves should actually run at. The ceiling is a cliff,
# not a cruising speed - there is nothing to gain from crossing the
# bed at the exact rate where step loss begins.
DEFAULT_TRAVEL_FEED = 1800.0

# The stage sweeps its step rate continuously, so a gradient is one move
# rather than a chain of constant-feed segments. These survive only because
# gradient_segments() is still useful for previewing a profile.
GRADIENT_SEGMENT_MM = 2.0
MAX_GRADIENT_SEGMENTS = 60
# Acceleration takes distance. A pass that starts and stops on the coated
# span cannot be at the requested feed at its own ends, whatever the
# acceleration - so passes run in from outside the span and out past it,
# and the gradient is defined only over what lies between. Replaced by the
# stage's own acceleration once connected.
DEFAULT_ACCEL = 800.0
JUNCTION_MMS = 2.0

# Which way the jog pad moves the coordinate. The pad's quadrants are
# physical directions as the operator sees the machine; the home corner is
# back-right and coordinates grow away from it, so on this stage pressing
# "right" or "back" makes the number smaller. Set one of these False if that
# axis turns out to jog backwards.
# False because the pad was written for this coordinate frame in the first
# place: with the work area running negative from a back-right home, "right"
# and "back" really are the increasing directions.
JOG_INVERT_X = False
JOG_INVERT_Y = False
MAX_FEED_Z = 0.0  # no Z axis on this machine
# The cup is a fixture, not a per-job setting, so it persists on its own
# rather than only inside a saved recipe.
# The stage gets its own settings file, with the Shapeoko's read once as a
# fallback so an existing cup and feed ceiling carry over on first run.
SETTINGS_FILE = Path.home() / ".spray_stage.json"
LEGACY_CUP_FILE = Path.home() / ".shapeoko_spray.json"
Z_FEED = 0.0  # no Z axis: the spray standoff is fixed
# $132 says 95 mm of Z travel, but the last few millimetres are not usable in
# practice — the head complains at -95 and is happy at -90. Keep clear of it.
Z_BOTTOM_MARGIN = 5.0
# Below this, a Z move is not worth sending at all.
Z_NOOP_MM = 0.005

@dataclass
class Envelope:
    """Reachable travel, in stage machine coordinates.

    Zero is the home corner - back-right - and the numbers grow as the head
    moves away from it, so everything reachable is positive. That is the
    opposite sign convention from the Shapeoko, whose coordinates ran
    negative from the same corner; the machine reports these directly, so
    no translation happens anywhere in this module.

    lo_* is the near end, just clear of the homing switches, and hi_* is the
    far end of travel. There is no Z: the spray standoff is fixed, and the
    Z members are kept only so the three-axis plumbing above them - the
    position readout, stored cup heights, jog_target - needs no changes.
    """

    min_x: float = DEFAULT_MIN_X  # park point, just off the X switch
    max_x: float = DEFAULT_MAX_X  # far end of X travel, less an edge margin
    min_y: float = DEFAULT_MIN_Y
    max_y: float = DEFAULT_MAX_Y
    max_z: float = DEFAULT_MAX_Z
    pulloff: float = DEFAULT_PULLOFF
    z_margin: float = Z_BOTTOM_MARGIN
    max_rate_xy: float = DEFAULT_MAX_RATE_XY  # measured step-loss cliff
    max_feed_xy: float = DEFAULT_MAX_FEED_XY  # operator ceiling, set in the GUI
    accel_x: float = DEFAULT_ACCEL
    accel_y: float = DEFAULT_ACCEL
    junction: float = JUNCTION_MMS

    @property
    def lo_x(self) -> float:
        return self.min_x

    @property
    def hi_x(self) -> float:
        return self.max_x

    @property
    def lo_y(self) -> float:
        return self.min_y

    @property
    def hi_y(self) -> float:
        return self.max_y

    @property
    def lo_z(self) -> float:
        return 0.0

    @property
    def hi_z(self) -> float:
        return 0.0

    def accel(self, axis: str) -> float:
        return self.accel_x if axis.upper() == "X" else self.accel_y

    def lead(self, feed: float, axis: str) -> float:
        """Run-up a pass needs before it is at `feed`, in mm.

        v^2 = 2*a*d from the junction speed, matching the firmware exactly,
        plus half a millimetre so the coated span never begins at the very
        point the ramp ends. This is the number that decides whether a
        gradient's fast end is real or merely requested.
        """
        v = feed / 60.0
        v0 = self.junction
        return max(0.0, (v * v - v0 * v0) / (2.0 * self.accel(axis))) + 0.5

    def clamp_error(self, label: str, x: float, y: float) -> list[str]:
        errs = []
        if not self.lo_x <= x <= self.hi_x:
            errs.append(f"{label} X={x:g} outside [{self.lo_x:g}, {self.hi_x:g}]")
        if not self.lo_y <= y <= self.hi_y:
            errs.append(f"{label} Y={y:g} outside [{self.lo_y:g}, {self.hi_y:g}]")
        return errs

    def clamp_error_xyz(self, label: str, x: float, y: float, z: float) -> list[str]:
        return self.clamp_error(label, x, y) + self.clamp_error_z(label, z)

    def clamp_error_z(self, label: str, z: float) -> list[str]:
        """No Z axis, so nothing to be outside of. Kept so that callers -
        and stored cup heights from the Shapeoko - keep working."""
        return []
