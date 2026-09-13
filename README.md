# limelight-yellow-waffle-ball

A Limelight 3A **SnapScript** pipeline that detects a yellow waffle ball, plus the
robot-side code to read what it publishes.

| file | what it is |
|---|---|
| `snapscript/yellow_waffle_ball.py` | runs **on the camera** — paste into the Python tab of a Limelight pipeline |
| `robot/read_llpython.py` | runs **on the robot** — reads the `llpython` array over NetworkTables |

## Why it isn't a plain blob detector

The waffle lattice means the ball is not a solid color patch. A raw HSV mask comes
back as a ring of disconnected fragments, every one of them too small to survive an
area filter. Two things fix that:

- a **morphological close** sized to the hole diameter, which welds the lattice into
  one silhouette before contour finding;
- **convex-hull** shape tests instead of perimeter circularity, because a lattice
  edge has an enormous, noisy perimeter that wrecks the usual `4*pi*A/P^2` metric.

The detector keeps the largest candidate that passes both a porosity test
(`contour area / hull area`) and a roundness test (`hull area / pi*r^2`).

## Output contract

`llpython`, an 8-element array on the `limelight` NetworkTable:

| index | value |
|---|---|
| 0 | valid flag, 1 when a ball passed every shape test |
| 1 | `tx` degrees, + is right of crosshair |
| 2 | `ty` degrees, + is above crosshair |
| 3 | distance in meters, from apparent ball diameter |
| 4, 5 | center `cx`, `cy` in px |
| 6 | radius in px |
| 7 | circularity 0..1, usable as a confidence gate |

## Tuning — three things that decide whether this works on a field

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

## Status

Syntax-checked only. It has **not** been run against a real camera or real footage —
the constants above are starting points, not measured values.
