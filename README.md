# limelight-yellow-waffle-ball

A Limelight 3A **SnapScript** pipeline that detects a yellow waffle ball, a four-motor
**mecanum drivebase** for FTC, and the robot-side glue to read what the pipeline
publishes.

| file | runs on | control system |
|---|---|---|
| `snapscript/yellow_waffle_ball.py` | the **camera** — paste into the Python tab of a Limelight pipeline | either |
| `ftc/MecanumDrivebase.java` | the Control Hub | FTC |
| `ftc/MecanumTeleOp.java` | the Control Hub | FTC |
| `ftc/LimelightBallTracker.java` | the Control Hub | FTC |
| `robot/read_llpython.py` | the roboRIO | **FRC** — NetworkTables, not FTC |
| `tools/selftest.py`, `tools/synth_waffle.py` | your laptop | neither — offline test harness |

FTC and FRC read the Limelight through completely different plumbing. FTC treats it as
a hardware device in the Robot Configuration and gets the pipeline's output from
`LLResult.getPythonOutput()`; FRC reads a NetworkTables entry. Both readers are here —
**use `ftc/LimelightBallTracker.java` on an FTC robot** and ignore `robot/read_llpython.py`.

## The vision pipeline

### Why it isn't a plain blob detector

The waffle lattice means the ball is not a solid color patch. A raw HSV mask comes
back as a ring of disconnected fragments, every one of them too small to survive an
area filter. Two things fix that:

- a **morphological close** sized to the hole diameter, which welds the lattice into
  one silhouette before contour finding;
- **convex-hull** shape tests instead of perimeter circularity, because a lattice
  edge has an enormous, noisy perimeter that wrecks the usual `4*pi*A/P^2` metric.

The detector keeps the largest candidate that passes both a porosity test
(`contour area / hull area`) and a roundness test (`hull area / pi*r^2`).

### Output contract

`llpython`, an 8-element array:

| index | value |
|---|---|
| 0 | **source**: 0 nothing, 1 contour path, 2 Hough fallback |
| 1 | `tx` degrees, + is right of crosshair |
| 2 | `ty` degrees, + is above crosshair |
| 3 | distance in meters, from apparent ball diameter |
| 4, 5 | center `cx`, `cy` in px |
| 6 | radius in px |
| 7 | confidence 0..1 — circularity on the contour path, disk fill on the Hough path |

Index 0 is a **source code, not a boolean** — `if (source != 0)` still reads as "valid",
but source 2 means the ball was fused into a same-hue blob (a bumper, a wall, another
ball) and a Hough pass recovered the circle from inside it. That is a real detection on
a degraded path: both readers in this repo gate it looser (0.55 vs 0.75) and flag it, and
its range is worth less than a clean contour's.

Index 7 measures two different things depending on the path, which is why it gets two
different gates rather than one.

### Tuning — three things that decide whether this works on a field

**`CLOSE_K` is the whole ballgame.** It must exceed the waffle hole width in pixels
at your *farthest* useful range, or the ball fragments into a cloud of small contours
and every one fails `MIN_AREA`. Measured on synthetic lattice frames: with a 5.2 px
hole, `CLOSE_K` of 3 and 5 both **miss** and 7 upward detect — so the working margin is
roughly **1.4x the hole width**, not 1.0x. At a 15.6 px hole every value from 3 to 21
detected, because a ball that large stays connected regardless. The constraint binds at
the far end of your range only. Tune it by watching the mask in the Limelight web UI at
your actual working distance.

**Order the morphology speck-open → close → open.** Closing first drags nearby yellow
noise into the ball's convex hull and inflates the radius; measured 9.2% radius error
that way against 4.8% with a 3 px open in front. Since distance is derived from radius,
that error lands directly on your range estimate.

**Take HSV bounds from real field footage, not from a color picker.** Yellow under
arena LEDs desaturates toward white at the specular highlight and toward orange at the
shadow terminator. Widen `S`/`V` before you widen `H` — widening `H` is what starts
pulling in orange and green field elements.

**Distance is only as good as `BALL_DIAMETER_M`**, and it degrades once the ball is
partially occluded, since the min-enclosing circle shrinks with the visible silhouette.
If you need range for a pickup, trust `tx` for steering and use a floor-plane projection
off `ty` and the camera mount height/angle instead — that doesn't care about occlusion.

`HFOV_DEG` is set for the LL3A stock lens; the focal length in pixels is derived from
the frame width on the first frame, so it follows whatever capture resolution the
pipeline is set to.

## The FTC drivebase

Drop the three `ftc/*.java` files into `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
(or paste them into OnBot Java). Robot Configuration names expected:

```
front_left   front_right   back_left   back_right      DcMotor
imu                                                    built into the Control Hub
limelight                                              Ethernet Device -> Limelight3A
```

`MecanumDrivebase` gives you `driveRobotCentric(forward, strafe, turn)` and
`driveFieldCentric(...)`, both taking -1..1. The inputs are normalised together by
`max(|f| + |s| + |t|, 1)`, so a full-stick diagonal keeps its heading instead of
clipping into a curve. `STRAFE_GAIN` scales the strafe axis up because mecanum rollers
make sideways travel weaker than forward travel at equal power — tune it by driving a
taped square.

`MecanumTeleOp` is a working OpMode: left stick translates, right stick x turns, right
trigger is a precision creep, `options` re-zeros the field-centric heading, `back`
toggles field- vs robot-centric.

### Bring it up on blocks, in this order

1. Left stick forward — **all four wheels must spin forward.** A wheel going backwards
   is a reversed motor, not a math problem; fix the direction in `MecanumDrivebase`.
2. Left stick right — the wheels must form an **X pattern** seen from above. If the
   robot rotates instead, two motors are swapped in the Robot Configuration.
3. Only then put it on the floor. A chassis with one roller set mounted backwards
   drives fine forward and crabs sideways on every turn, which looks like a software
   bug and is not one.

## Running it without a camera

`tools/` holds an offline harness. It synthesises lattice-ball frames and runs the real
pipeline over them, so the geometry and morphology can be exercised on a laptop:

```
python3 -m venv .venv
./.venv/bin/pip install -r tools/requirements.txt
cd tools && ../.venv/bin/python selftest.py [--dump DIR]
```

`--dump` writes the annotated frames, which is the fastest way to see *why* something
was rejected — failed candidates are outlined in red, the Hough fallback draws orange.

Be clear about what this proves. The synthetic frames reproduce the one property that
makes a waffle ball hard — a porous lattice silhouette — plus shading, a specular
highlight, sensor noise, and four distractors (yellow tape, an orange ball, yellow
specks, and a large same-hue bumper slab). They reproduce none of what makes a real
field hard: rolling shutter, motion blur, mixed color temperature, a ball half-occluded
behind a robot. **Passing means the geometry and the morphology are right. It says
nothing about whether the HSV bounds are right** — those need real footage.

## Status

**Python — exercised locally against synthetic frames, `selftest.py` passes clean:**

- detected across apparent radii of 10–130 px (3.3 m down to 0.25 m at the placeholder
  ball diameter); center within 5% of radius, radius error ≤ 4.8%
- all four distractor classes rejected, including a bumper slab larger in area than the
  ball — the detector scores every contour over the area floor rather than the top N by
  area, because at 1.6 m the ball was only the *third*-largest contour in frame
- a ball fused to a same-hue slab fails the contour path, as it should, and the Hough
  fallback recovers it to within 3.2 px and −2.2% radius

Three bugs the harness found and that are now fixed: `MIN_AREA` of 300 px² silently cut
off everything past ~2.5 m; top-3-by-area ranking could let distractors starve the real
ball; and the focal length was cached in a bare global, so changing the pipeline's
capture resolution would have kept a stale value.

Still unverified on hardware: HSV bounds, `BALL_DIAMETER_M`, and the Hough fallback's
CPU cost on the Limelight's own processor.

**Java — compiles clean** against a stubbed FTC SDK surface (all four classes, `javac`
exit 0). Not built against the real SDK, not run on a Control Hub.
