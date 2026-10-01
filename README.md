# Spray Coater — SKR Pico XY stage

An XY stage that sprays gradients of paint onto microscope slides. Two
motors drive the Y gantry in lockstep, one drives the X crossbeam, and
there is no Z: the spray standoff is fixed and set by hand.

The GUI and recipe planner came from the Shapeoko spray program; the
machine under them is new.

## Running it

    ./run.sh            # creates .venv on first run, then launches the GUI
    ./push.sh           # copies the firmware to the Pico and resets it

The first `./run.sh` builds the virtualenv and installs pyserial and
mpremote, so it takes a minute. After that it is immediate.

## Layout

| path | what it is |
| --- | --- |
| `spray_stage/spray_gui.py` | the GUI, recipe table, script mode and planner |
| `spray_stage/pico_controller.py` | serial transport; speaks GRBL up, stage dialect down |
| `spray_stage/stage.py` | travel, feeds, acceleration — the machine model |
| `spray_stage/check_recipe.py` | dry-run a recipe and print the G-code, no hardware |
| `spray_stage/migrate_recipes.py` | move Shapeoko recipes onto this bed |
| `pico_stage/` | what runs on the board — see its own README |
| `patterns/` | recipes, already migrated |

## Coordinates

Zero is the home corner, back-right, and the work area runs **negative**
from it — the same convention the Shapeoko used, so recipes and muscle
memory carry over. Travel is X −94…−3 and Y −139…−3; homing parks at
−6, which leaves room to jog toward home afterwards.

The firmware counts *positive* millimetres from home, because that is the
natural frame for soft limits and lead distances. The sign flip happens in
`PicoController`, in two methods, and nothing above it sees a positive
coordinate.

## Limits worth knowing

| | X (crossbeam) | Y (gantry) |
| --- | --- | --- |
| travel | 91 mm | 136 mm |
| top feed without step loss | 2400 mm/min | 2400 mm/min |
| acceleration in use | 800 mm/s² | 800 mm/s² |
| homing repeatability | 0.001 mm | 0.005 mm |

2400 mm/min is a cliff, not a soft limit — 50 mm/s loses about 0.16 mm per
out-and-back. A feed above it is **refused rather than clamped**, because
clamping would flatten the fast end of a gradient and change the coating
while appearing to work. Rescale the whole recipe instead; the migrator
does this and reports the factor.

## Gradients

A gradient pass is one move, not a chain of constant-feed segments: the
firmware sweeps its step rate continuously while the move runs. Passes also
run in from before the coated span and out past it, so the acceleration
ramps land off the slide — a pass that starts and stops on the span cannot
be at the requested feed at its own ends, whatever the acceleration.

`check_recipe.py` prints exactly what will be sent:

    .venv/bin/python -m spray_stage.check_recipe patterns/2026Aug26Gradient_Thickness.json

## Tests

    .venv/bin/python -m pytest tests/ -q

122 tests, about a tenth of a second. They cover the planner, the script
language, the gradient maths and the transport's translation layer — all
pure logic, no hardware and no display. `conftest.py` stubs Tkinter where
it is missing, so the suite runs on a headless machine too.

Worth running after any edit to `spray_gui.py`, `stage.py` or
`pico_controller.py`: the things they catch — a sign flip the wrong way, a
lead distance that stops matching the firmware's, a modal `G90` that hides
a Z word from the stripper — are all faults that look fine from the GUI
and go wrong on the slide.
