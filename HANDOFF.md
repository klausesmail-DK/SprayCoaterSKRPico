# Spray coater — orientation for a new session

Read this first, then the two READMEs it points at. It covers what the
program is, where the pieces are, what must not be broken, and what is
currently loose.

## The machine

An XY stage that sprays gradients of paint onto microscope slides.

- **Y** — a gantry driven by **two** motors in lockstep.
- **X** — a crossbeam on one motor.
- **No Z.** The spray standoff is fixed and set by hand. The Z plumbing
  that survives in the code is deliberate; see *Why Z is still in there*.

Controller is a BIGTREETECH **SKR Pico v1.0** running MicroPython, over USB
serial at 115200. 1/16 microstep, Tr6x4 leadscrews, 800 steps/mm, 24 V,
0.40 A.

The GUI and recipe planner were ported from the earlier Shapeoko spray
program. The machine underneath them is new; that port is the source of most
of the sharp edges below.

## Where everything is

Active project: **`~/Projects/SprayCoaterSKRPico`** (git, 2 commits).

| path | what it is |
| --- | --- |
| `spray_stage/spray_gui.py` | 2600 lines: GUI, recipe table, script mode, planner |
| `spray_stage/pico_controller.py` | serial transport — GRBL upward, stage dialect downward |
| `spray_stage/stage.py` | the machine model: travel, feeds, acceleration |
| `spray_stage/check_recipe.py` | dry-run a recipe, print the G-code, no hardware |
| `spray_stage/migrate_recipes.py` | move Shapeoko recipes onto this bed |
| `pico_stage/skr_pico_stage.py` | 1300 lines of firmware — lives on the Pico |
| `pico_stage/main.py` | runs `serve()` at boot, with a 3 s Ctrl-C escape window |
| `patterns/` | recipes, already migrated |
| `tests/` | 122 tests, pure logic, no hardware, no display |

`~/Projects/shapeoko` is the **old** program. It is kept for reference only;
what moved is parked in its `_moved_to_SprayCoaterSKRPico/`. Do not edit it
expecting the stage to change.

## Running it

    ./run.sh      # builds .venv on first run, then launches the GUI
    ./push.sh     # copies the firmware to the Pico and resets it
    ./check.sh    # what's on the board + does the firmware import
    ./reset.sh    # restart the board so serve() owns the serial line

**The one recovery fact worth memorising:** `mpremote` sends Ctrl-C when it
attaches, which stops `serve()`, and nothing restarts it. So after *any*
`mpremote` command other than `reset` — `check.sh` included — run
`./reset.sh`. The symptom otherwise is a GUI that opens the port and then
gets no reply, which looks like a dead board.

Tests and a dry run:

    .venv/bin/python -m pytest tests/ -q
    .venv/bin/python -m spray_stage.check_recipe patterns/2026Aug26Gradient_Thickness.json

## The architecture, in one paragraph

The GUI plans a recipe into GRBL-flavoured G-code, exactly as it did for the
Shapeoko. `PicoController` is the only thing that knows the machine is not a
Shapeoko: it speaks GRBL upward (`$H`, `$J=`, realtime `?`, `0x85`, `0x21`,
a `G21 G90 M5` preamble) and the firmware's much smaller dialect downward.
`stage.py` holds the machine's dimensions so that a test, the migrator or a
script can ask about the machine without importing Tkinter. The firmware
below that owns motion, homing and the soft limits.

### The sign flip is a single boundary

Above `PicoController`, coordinates are **negative**: zero is the home
corner (back-right) and the work area runs away from it. That is the
Shapeoko convention, which is what every recipe on disk and all of Klaus's
muscle memory is in. Below it, the firmware counts **positive** millimetres
from home, which is the natural frame for soft limits and lead distances.

The flip happens in **two methods** of `PicoController` and nowhere else.
Nothing in the GUI ever sees a positive coordinate, and nothing in the
firmware ever sees a negative one. Resist any change that spreads the
conversion.

Travel, as the operator sees it: **X −94…−3, Y −139…−3**. Homing parks at
**−6**, not on the soft limit, so there is still room to jog toward home
afterwards — parking on the limit reads as a broken jog pad.

### What else the transport translates

- `?`, `$H`, `G0`, `G1`, `G4` pass straight through.
- `G21 G90 M3 M5 M8 M9` are silently acked — absolute mm is all this machine
  does, and there is no spindle or coolant.
- **Z words are dropped.** Status reports Z as 0.
- `$J=` is rewritten as an absolute `G1`; the firmware has no jog and no
  relative mode, so the transport tracks position itself.
- **Continuous jog** runs as a chain of 1 mm absolute moves on a worker
  thread. Cancel is then "stop sending the next chunk", which preserves the
  step count and keeps the machine homed. `JOG_CHUNK_MM` is both the
  granularity and the stop latency.
- `0x85` stops the chunk chain, position intact. `0x21`/`0x18` send a real
  `!` abort — position **is** lost, the firmware goes to Alarm, and the GUI
  must re-home. That is correct behaviour for STOP.
- `$$` is parsed into limits, feeds and acceleration, which replace the
  `stage.py` defaults once connected.

### The one thing that is not a translation

GRBL acks a line when it has **buffered** it. This firmware has no
look-ahead buffer and acks when the move has **completed**. So `send` blocks
for the whole traverse, and its timeout must cover the longest single move,
not the longest queueing delay. This is why hold-to-move goes through
`start_jog` on the transport's own thread: a blocking send held the I/O lock
for the entire traverse, so the release meant to cancel it queued up behind
the move it was cancelling — the button looked dead and the head ran to the
limit.

## Invariants — the faults that look fine and go wrong on the slide

1. **2400 mm/min is a cliff, not a soft limit.** Both axes hold ±0.006 mm to
   40 mm/s and lose ~0.157 mm per out-and-back at 50 mm/s. A feed above the
   ceiling is **refused, never clamped** — clamping would flatten the fast
   end of a gradient and change the coating while appearing to work. Rescale
   the whole recipe instead; the migrator does this and reports the factor.
   1800 mm/min is the working traverse.
2. **A gradient is one move, not a chain of segments.** GRBL holds one feed
   per move, so the Shapeoko program chopped each pass into 2 mm
   constant-feed pieces — fifteen rate changes per pass. This firmware sweeps
   the PIO clock divider *while the move runs*, a few hundred updates per
   second. Step **count** comes from a FIFO, so position never depends on
   timing; step **rate** is therefore free. `gradient_segments()` survives
   only for previewing a profile.
3. **Passes run in before the span and out past it.** A pass that starts and
   stops on the coated span cannot be at the requested feed at its own ends,
   at any acceleration. Feed holds flat across the lead zones and the
   gradient is defined only over what lies between, so the ramps land off
   the slide. At 800 mm/s² the lead is 1.0 mm. The host's `Envelope.lead()`
   and the firmware's `lead_needed()` must agree — if they drift, the fast
   end of a gradient becomes merely *requested*.
4. **Gantry lockstep is enforced in firmware.** Both Y state machines live
   on PIO0, share a clock divider, and are started and stopped by a single
   write to PIO0's CTRL register so they begin on the same clock edge.
   Nothing may ever enable or disable one without the other — which is why
   every start and stop goes through `_sm_enable()`.
5. **Thickness vs velocity gradients.** "Velocity" is linear in speed;
   "thickness" is linear in deposited coating, which needs the reciprocal of
   speed to be linear, because coating per unit length goes as 1/v.

The test suite exists mostly to catch these: a sign flip the wrong way, a
lead distance that stops matching the firmware's, a modal `G90` hiding a Z
word from the stripper. All of them look fine from the GUI. Run the tests
after any edit to `spray_gui.py`, `stage.py` or `pico_controller.py`.

## Why Z is still in there

`stage.py` keeps `Z_FEED`, `Z_BOTTOM_MARGIN`, `MAX_FEED_Z` and friends, and
`Envelope` still has `max_z`, `lo_z`, `hi_z` and `clamp_error_z`. The Z
*widgets* were removed from the GUI and the transport drops Z words, but the
variables stayed so the saved cup record, the position readout and
`_z_values()` keep working with three axes and Z pinned at zero.
`clamp_error_z` returns an empty list by design. Don't "finish the cleanup"
without checking the cup record still loads.

## Firmware command set

    $H              home X then Y, park at PARK_MM
    $$              limits, feeds and acceleration, for the host to read
    ?               <Idle|MPos:x,y> or <Alarm|MPos:none>
    G0 X.. Y..      rapid at the travel feed
    G1 X.. Y.. F..  feed move, F in mm/min
    G1 .. F.. E..   gradient: sweep F to E across the move
                    I.. lead-in mm, K.. lead-out mm, Q1 for thickness mode
    G4 P..          dwell, seconds
    M17 / M18       enable / disable drivers
    !               abort; position lost, so re-home

Every command answers `ok` or `error: ...`; anything else is informational.

Bench routines on the board, for when something mechanical changes:

    selftest('X')       state-machine reset path, 30 cycles, expect <0.01 mm
    show_profile('Y')   rate schedule for a gradient; nothing moves; want 0.00%
    home_all()
    sync_test('Y')      gantry marks — run after any acceleration change
    speed_test('Y')     re-measure the ceiling
    accel_test('Y')

## Recipes and script mode

Recipes are JSON in `patterns/`, already migrated to this bed. A recipe is a
list of entries; each is `line` (sprays one way, returns fast), `line_back`
(sprays both directions, and is what carries a gradient) or `dot` (parks on a
point).

Script mode layers a tiny language over the table:

    LOOP(1,3,5)=3        run those entries, in that order, three times
    LOOP(1,3-5)=2        ranges work too
    TRANSLATE(10;20)     shift everything after this, cumulatively
    ORIGIN               reset the shift
    WAIT=3               pause, seconds

Statements split on `;` and newlines but not inside brackets, because
`TRANSLATE` uses `;` as its own separator. `#` starts a comment.

The cup / waste position is a **fixture, not a per-job setting**, so it
persists in `~/.spray_stage.json` rather than inside a recipe. The Shapeoko's
`~/.shapeoko_spray.json` is read once as a fallback so the cup and feed
ceiling carried over on first run.

## Measured limits

| | X (crossbeam) | Y (gantry) |
| --- | --- | --- |
| travel | 91 mm | 136 mm |
| top feed without step loss | 2400 mm/min | 2400 mm/min |
| acceleration in use | 800 mm/s² | 800 mm/s² |
| homing repeatability | 0.001 mm | 0.005 mm |

800 mm/s² is adopted for margin, not because 1600 failed — 1600 read clean
on both axes. **None of this was measured with the spray head and
pressurised hoses fitted**, and hose drag opposes motion. If passes start
losing position once the head is on, drop acceleration first.

## Current state and loose ends

The port is done and the suite is green at 122 tests. The jog fixes
(cancellable hold-to-move, pad direction) and the negative-coordinate /
no-Z change are both applied; the `apply_jog_fix.py` and
`apply_signs_and_z.py` scripts that applied them are spent and can go.

Stale text, none of it affecting behaviour, all of it liable to mislead a
future session that trusts a docstring:

- **`spray_stage/spray_gui.py` module docstring** is still the Shapeoko's.
  It claims the reachable range is "X −845…−6, Y −850…−6, Z −90…−6", talks
  about Z establishing the standoff, and points at
  `docs/usage/spray-line-gui.md`, which does not exist in this repo.
- **`pico_stage/README.md`** file table points at `../shapeoko_controller/…`.
  The package is `spray_stage/`.
- **`spray_stage/stage.py`** keeps the old positive-frame comment paragraph
  immediately above the new negative-frame one, so the two read as
  contradicting each other. The code is right; the first paragraph is dead.
- **`skr_pico_stage.py` module docstring** says Y's acceleration "has not
  been measured and is left at the old conservative default". Both axes are
  configured at 800 mm/s² and the per-axis comments record Y as clean to
  1600. `ACCEL_MMSS = 50.0` is only the no-value fallback.
- `~/Projects/shapeoko/Claude outputs/` holds older copies of
  `skr_pico_stage.py` and `xy_stage.py`. Not the live firmware.

## Working notes

Both project folders are connected read/write, so edit them in place rather
than shipping patch scripts — that was a workaround from when they weren't.
Edit with a command or a short read-modify-write, never by re-typing a
file's contents from earlier tool output.

The venv in the repo is a macOS venv, so its `python` is not runnable from
the Linux side of the bridge; run the tests on the Mac.
