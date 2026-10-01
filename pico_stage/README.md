# Pico XY spray stage

Firmware for the BIGTREETECH SKR Pico v1.0 driving the two-motor Y gantry and
single-motor X crossbeam, plus the notes needed to drive it from the spray
program.

## Files

| file | where it lives |
| --- | --- |
| `skr_pico_stage.py` | on the Pico |
| `main.py` | on the Pico - runs `serve()` at boot |
| `../shapeoko_controller/pico_controller.py` | host: the transport |
| `../shapeoko_controller/stage.py` | host: travel, feeds, acceleration |
| `../shapeoko_controller/migrate_recipes.py` | host: Shapeoko recipes -> stage coordinates |
| `../shapeoko_controller/check_recipe.py` | host: dry-run a recipe, no hardware |

## Measured limits

| | X (crossbeam) | Y (gantry) |
| --- | --- | --- |
| travel | 3 - 94 mm | 3 - 139 mm |
| steps/mm | 800 (1/16 ustep, Tr6x4) | 800 |
| top feed without step loss | 2400 mm/min | 2400 mm/min |
| acceleration, clean at | 800 mm/s2 (1600 also passed) | 800 mm/s2 (1600 also passed) |
| homing repeatability | 0.001 mm over 3 | 0.005 mm over 5 |

2400 mm/min is a cliff, not a soft limit: 50 mm/s loses about 0.157 mm per
out-and-back on both axes. A feed above it is refused rather than clamped,
because clamping would flatten the fast end of a gradient and change the
coating while appearing to work.

Acceleration is set to half the highest rate that passed. None of these
numbers were measured with the spray head and pressurised hoses fitted, and
hose drag opposes motion - if passes start losing position once the head is
on, drop acceleration first.

## Command set

    $H              home X then Y, park at 3 mm, which becomes position 3
    $$              limits, feeds and acceleration, for the host to read
    ?               status: <Idle|MPos:x,y> or <Alarm|MPos:none>
    G0 X.. Y..      rapid at the travel feed
    G1 X.. Y.. F..  feed move, F in mm/min
    G1 .. F.. E..   gradient: sweep F to E across the move
                    I.. lead-in mm, K.. lead-out mm, Q1 for thickness mode
    G4 P..          dwell, seconds
    M17 / M18       enable / disable drivers
    !               abort; position is lost, so re-home

A command answers `ok` or `error: ...`. Anything else is informational.

## Why a gradient is one move

GRBL holds one feed per move, so the Shapeoko program approximated a ramp by
chopping each pass into 2 mm constant-feed segments. This firmware sweeps the
PIO clock divider while the move runs, so the ramp is continuous and arrives
as a single command. The step count comes from a FIFO rather than from
timing, so the rate can be changed freely without ever costing position.

## Why passes run past their ends

A pass that starts and stops on the coated span cannot be at the requested
feed at its own ends - it has to accelerate and decelerate somewhere, at any
acceleration. So the carriage runs in from `lead` before the span and out to
`lead` past it, the feed holds flat across those zones, and the gradient is
defined only over what lies between. At 800 mm/s2 the lead is 1.0 mm.

`show_profile('Y')` simulates a pass and reports the worst shortfall inside
the coated span; it should read 0.00%.

## Bringing it up

    selftest('X')       state-machine reset path: 30 cycles, expect < 0.01 mm
    show_profile('Y')   rate schedule for a gradient, nothing moves
    home_all()
    sync_test('Y')      gantry marks, after any acceleration change
    speed_test('Y')     re-measure the ceiling if anything mechanical changes
