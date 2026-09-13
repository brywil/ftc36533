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
| 0 | valid flag, 1 when a ball passed every shape test |
| 1 | `tx` degrees, + is right of crosshair |
| 2 | `ty` degrees, + is above crosshair |
| 3 | distance in meters, from apparent ball diameter |
| 4, 5 | center `cx`, `cy` in px |
| 6 | radius in px |
| 7 | circularity 0..1, usable as a confidence gate |

### Tuning — three things that decide whether this works on a field

**`CLOSE_K` is the whole ballgame.** It must exceed the waffle hole width in pixels
at your *farthest* useful range, or the ball fragments into a cloud of small contours
and every one fails `MIN_AREA`. Too large and two adjacent balls merge into one blob.
Tune it by watching the mask in the Limelight web UI at your actual working distance.

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

## Status

- **Python**: syntax-checked only. It has **not** been run against a real camera or
  real footage — the constants are starting points, not measured values.
- **Java**: compiles clean against a stubbed FTC SDK surface (all four classes). It has
  **not** been built against the real SDK or run on a Control Hub.
