# ftc36533

FTC team 36533 robot code for **BIOBUZZ** (2026-27). Right now that is a four-motor
mecanum drivebase and a Limelight 3A pipeline that finds POLLEN, but this repo is the
general home for the team's code — new subsystems go here.

The two BIOBUZZ scoring elements, from the Section 16 glossary:

| element | spec | why it matters here |
|---|---|---|
| **POLLEN** | 2.8 in. (7.1 cm) Gopher ResisDent™ polyethylene balls **in yellow** | what the pipeline detects; the lattice is why it isn't a blob detector |
| **NECTAR** | ~3.6 in. (9.1 cm), red or blue | bigger and a different hue, so the yellow gate rejects it for free |

| path | runs on |
|---|---|
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` | the Control Hub — OpModes and subsystems |
| `snapscript/ball_detector.py` | the **camera** — paste into the Python tab of a Limelight pipeline |
| `tools/` | your laptop — offline vision harness, no camera needed |
| `install.sh`, `build.sh` | your laptop — toolchain bootstrap and APK build |
| `GETTING_STARTED.md` | **start here if you are new** — plain-language setup and driving guide |

> **New to this?** Read [GETTING_STARTED.md](GETTING_STARTED.md) first. It explains
> the vocabulary, how to get the code onto the robot, and what to do when something
> breaks. This README is the reference: it records *why* things are the way they are
> and what was measured. Both are meant to be readable without a degree — if a term
> here isn't explained, that's a bug, tell us.

## Quick start

On a machine with nothing installed:

```bash
git clone git@github.com:brywil/ftc36533.git
cd ftc36533
./install.sh          # then ./build.sh
```

`install.sh` installs the system packages, a JDK the Android Gradle Plugin accepts,
the Android SDK command-line tools, the FTC SDK itself (pinned to a tag), links this
repo's OpModes into it, and builds a Python venv for the vision harness. It finishes
by building the APK and running the vision self-test, so a successful run means the
toolchain actually works rather than merely being present.

It is re-runnable — every step checks for its own result first.

```
./install.sh --dry-run        print every command, change nothing
./install.sh --vision-only    just the Python side: no sudo, no Android SDK
./install.sh --skip-packages  you installed the system packages yourself
./install.sh --sdk-tag v12.0  build against a different FTC SDK release
./install.sh -h               all flags
```

Only Debian/Ubuntu package installation is automated. On anything else the script
prints exactly which packages it needs and stops, rather than guessing at another
package manager's names.

```
./build.sh            build the APK
./build.sh install    build and push to a connected hub (adb connect 192.168.43.1:5555 over Wi-Fi)
./build.sh clean      wipe build outputs
./build.sh <task>     any other gradle task
```

The FTC SDK is cloned into `ftc-sdk/` and is **not** tracked here — it is pinned by tag
in `install.sh`, so re-running gets everyone the same one. Your OpModes are symlinked
into it, so editing a file in `TeamCode/` is picked up by the next build with no sync
step to forget.

## The drivebase

Robot Configuration names expected on the Control Hub:

```
front_left   front_right   back_left   back_right      DcMotor
intake     lift_left     lift_right                    DcMotor
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

### Attachments

`AttachmentMotors` owns one intake and two lift motors (names `intake`, `lift_left`,
`lift_right`). They are simple open-loop spin motors — no encoders, no holding a
position. Each is looked up with `tryGet`, so a missing name does not stop the
drivebase working; `HardwareCheck` is where a wrong name shows up.

| control | does |
|---|---|
| right bumper | run the intake, hold to run |
| left bumper | run the intake reversed — clears a jam |
| dpad up / down | raise / lower the lift, hold to run |

Both are **hold-to-run**, so the safe failure is always "let go". A lift motor that
fights its twin is a reversed motor: flip `LIFT_LEFT_REVERSE` / `LIFT_RIGHT_REVERSE`
in `AttachmentMotors.java`. Test on blocks before the lift can hit anything.

### Bring it up on blocks, in this order

1. Left stick forward — **all four wheels must spin forward.** A wheel going backwards
   is a reversed motor, not a math problem; fix the direction in `MecanumDrivebase`.
2. Left stick right — the wheels must form an **X pattern** seen from above. If the
   robot rotates instead, two motors are swapped in the Robot Configuration.
3. Only then put it on the floor. A chassis with one roller set mounted backwards
   drives fine forward and crabs sideways on every turn, which looks like a software
   bug and is not one.

## The vision pipeline

### Why it isn't a plain blob detector

POLLEN balls have holes all over them, like a wiffle ball. That breaks the obvious
approach, which would be "find the yellow blob". Search for yellow and you don't get a
circle — you get a ring of little disconnected yellow scraps with gaps between them,
and every scrap is too small to look like a ball.

Two things fix it:

- A **morphological close** — a standard image trick that fills in small gaps. Sized to
  the width of the holes, it welds the scraps back into one solid shape.
- **Convex-hull** shape tests. The convex hull is the shape you'd get by stretching a
  rubber band around the object. We test that instead of the outline itself, because a
  holey edge is long and jagged, and the usual roundness formula (`4*pi*area/perimeter^2`)
  gets wrecked by a jagged perimeter. The rubber-band shape ignores the jaggedness.

The detector keeps the biggest candidate that passes two tests: **how solid it is**
(contour area ÷ hull area — a lattice is holey, so this is allowed to be low) and **how
round it is** (hull area ÷ the area of a circle drawn around it).

### Output contract

The pipeline publishes 8 doubles, read on the robot by `LimelightBallTracker`:

| index | value |
|---|---|
| 0 | **what and how**, packed as `class * 10 + source` (see below) |
| 1 | `tx` degrees, + is right of crosshair |
| 2 | `ty` degrees, + is above crosshair |
| 3 | distance in meters, from apparent ball diameter |
| 4, 5 | center `cx`, `cy` in px |
| 6 | radius in px |
| 7 | shape score 0..1 — solidity × roundness |

| index 0 | meaning |
|---|---|
| 0 | nothing found |
| 11 / 12 | POLLEN, by contour / by Hough recovery |
| 21 / 22 | NECTAR red |
| 31 / 32 | NECTAR blue |

Any non-zero value means something was found, so the usual robot check is just
"is index 0 not zero". The robot can also write `llrobot[0]` to ask for one kind of
ball only — worth doing, since searching one colour is about three times cheaper
than searching three and the Limelight's processor is not fast.

### What separates a ball from the background

Two findings from photographing real balls, both of which overturned the obvious guess.

**Saturation does the work, not hue.** Under warm indoor light the cream curtain and
the wall measured median hue 22 — the *same hue as a POLLEN ball*. Hue cannot separate
them at all. What separates them is that a ball is vividly coloured and a wall is not:
raising the saturation floor from 90 to 110 removed two thirds of the background and
cost no detections. The hue ranges are set wide only to survive a change of lighting.

**Shape separates a red ball from a red shirt.** They are the same hue *and* the same
saturation, so no colour gate can help. Measured: the shirt scores solidity 0.72 and
roundness 0.71 — it squeaks past both floors individually, because a torso cropped by
the frame is passably round and passably solid. A real ball is strongly one or the
other (the red ball measured 0.96 × 0.79). So the gate is their **product**,
`MIN_SHAPE = 0.62`: "good at both" is a ball, "mediocre at both" is a person.

Two settings are per-class for the same reason, and the reason is clutter, not the ball:

- **`MIN_AREA_BY_CLASS`** — yellow has almost nothing competing with it indoors, so
  POLLEN keeps a low floor and keeps its range. Red has a shirt, which sheds small
  round fragments that pass every shape test; one was reported as a ball 1.95 m away.
  Red pays for its noise with range.
- **`HOUGH_BY_CLASS`** — the Hough recovery genuinely rescued two *blue* balls fused
  together, which is what it was built for. On red it fabricated balls in three photos
  that contained no red ball at all, including a photo of two yellow ones. From a
  colour mask alone there is no way to distinguish "a ball fused to something" from
  "an arbitrary circle carved out of a big region", so it is allowed where clutter is
  scarce and refused where it is not.

**Known limitation:** a red ball touching a large red object cannot be recovered. In
our test photo the ball merged with the shirt and roundness fell to 0.48. That test was
deliberately harder than a real field, where a NECTAR ball is unlikely to be resting
against something big and red.

### Tuning — three things that decide whether this works on a field

**`CLOSE_K` is the whole ballgame.** It must exceed the waffle hole width in pixels at
your *farthest* useful range, or the ball fragments into a cloud of small contours and
every one fails `MIN_AREA`. Measured on synthetic lattice frames: with a 5.2 px hole,
`CLOSE_K` of 3 and 5 both **miss** and 7 upward detect — so the working margin is
roughly **1.4x the hole width**, not 1.0x. At a 15.6 px hole every value from 3 to 21
detected, because a ball that large stays connected regardless. The constraint binds at
the far end of your range only.

**Order the morphology speck-open → close → open.** Closing first drags nearby yellow
noise into the ball's convex hull and inflates the radius; measured 9.2% radius error
that way against 4.8% with a 3 px open in front. Distance is derived from radius, so
that error lands directly on your range estimate.

**Take HSV bounds from real field footage, not from a color picker.** Yellow under arena
LEDs desaturates toward white at the specular highlight and toward orange at the shadow
terminator. Widen `S`/`V` before you widen `H` — widening `H` is what starts pulling in
orange and green field elements.

`BALL_DIAMETER_M` is 0.071 m, the official POLLEN diameter. Distance scales linearly off
it, so this is the one constant you must not eyeball. `HFOV_DEG` is the LL3A stock lens,
and focal length in px is derived from the frame width, so it follows whatever capture
resolution the pipeline is set to.

### POLLEN is small — know your range budget

At 2.8 in., POLLEN goes under the detector's area floor sooner than you would guess.
Apparent radius and contour area against range, by Limelight capture width:

| range | 640x480 | 1280x960 |
|---|---|---|
| 0.30 m | r 43.6 px, area 5961 | r 87.1 px, area 23845 |
| 0.50 m | r 26.1 px, area 2146 | r 52.3 px, area 8584 |
| 1.00 m | r 13.1 px, area 537 | r 26.1 px, area 2146 |
| 1.50 m | r 8.7 px, area 238 | r 17.4 px, area 954 |
| 2.00 m | r 6.5 px, area 134 | r 13.1 px, area 537 |
| 3.00 m | r 4.4 px, **area 60 — under `MIN_AREA`** | r 8.7 px, area 238 |

So at 640x480 the practical ceiling is roughly **1.5 m**, and the way to buy range is
capture resolution, not a lower `MIN_AREA` — dropping the floor lets specks back in.
Doubling the width doubles the focal length and therefore the apparent radius.

## HIVE localization (proof of concept)

| file | what |
|---|---|
| `TeamCode/.../HiveGeometry.java` | every constant, split into quoted-from-manual and must-measure |
| `TeamCode/.../HiveTracker.java` | cluster detection plus the settled/mid-swing gate |
| `TeamCode/.../HiveBenchOpMode.java` | bench bring-up, runs on a single lifted CELL |
| `tools/hive_gate_sim.py` | sweeps the arc and checks the gate thresholds offline |

### Why the HIVE can be localized against

It is a seesaw on a fixed pivot with two stable endpoints held by dampers, so it
**never rests in between**. A settled HIVE puts its tag clusters at one of two known
poses; anything else is a swing in progress. The danger is that a mid-swing solve
does not fail — it returns a confident pose against a cluster that is not where the
map says it is, and your position estimate teleports.

Both CELLS face downward at all times and are visible together, so the cluster ID
tells you *which CELL*, never *which state*. State comes from which CELL is currently
low.

### The gate

Two independent, gravity-referenced quantities, which must agree:

- **tilt** — the tag plane sits 30° off horizontal when settled and sweeps 60° when not
- **height** — the tag plane is at 25.5 in. or 44.3 in., **18.8 in. apart**, nothing legitimate between

Neither depends on the robot's estimated pose, so neither can be corrupted by the
thing they protect. `hive_gate_sim.py` sweeps the whole arc: with ±12° and ±6 in.
tolerances, **60% of the travel is rejected** and each endpoint keeps 12° of slack.
The two tests never disagree geometrically — so a disagreement in the field means a
bad solve or a miscalibrated camera, which is why the tracker refuses rather than
picking a winner.

### Bench procedure — one CELL, no field required

Nothing here is field-referenced, so one lifted CELL with one cluster of four tags
runs every check.

1. **Calibrate.** `CAMERA_PITCH_DEG` and the sign of `ftcPose.pitch` cannot be derived,
   only observed. Hold the CELL at each endpoint, press A and B; the OpMode prints the
   constants to paste into `HiveGeometry`. It also checks that your two captures
   straddle zero and span ~60° — if the midpoint is off zero, your camera pitch is
   wrong by exactly that much, and every tilt reading is biased the same way (which
   presents as "always mid-swing", not as a calibration error).
2. **Prove the gate.** Swing the CELL by hand and watch SETTLED drop out in the middle
   and return at the ends.
3. **Map the envelope.** Walk the camera back with a tape measure and find where the
   cluster stops solving at 100%. That number decides where the camera gets mounted.

### What this does and does not do

It reports range, bearing, slot, and a settled verdict. It does **not** yet output a
field pose: `HiveGeometry.SLOT_POSES_MEASURED` is `false` because the four slot poses
have to be measured off a real field first. §9.9 says the Reference Holes are the
intended way to do that, which is also why the SDK ships every cluster at (0,0,0).

It uses the **FTC SDK vision pipeline with a USB webcam** (configured as `Webcam 1`),
not the Limelight — the SDK has native four-tag cluster fusion via
`getBioBuzzTagLibrary()`, and the 13 in. baseline across a cluster is what makes yaw
trustworthy on 3.25 in. tags. A Limelight path would need a hand-authored `.fmap` and
its own cluster fusion; worth doing later, not for a proof of concept.

## Running the vision code without a camera

```bash
./install.sh --vision-only
cd tools && ../.venv/bin/python selftest.py [--dump DIR]
```

`--dump` writes the annotated frames, the fastest way to see *why* something was
rejected — failed candidates are outlined red, the Hough fallback draws orange.

Be clear about what this proves. The synthetic frames reproduce the one property that
makes a waffle ball hard — a porous lattice silhouette — plus shading, a specular
highlight, sensor noise, and four distractors (yellow tape, an orange ball, yellow
specks, a large same-hue bumper slab). They reproduce none of what makes a real field
hard: rolling shutter, motion blur, mixed color temperature, a ball half-occluded behind
a robot. **Passing means the geometry and the morphology are right. It says nothing
about whether the HSV bounds are right** — those need real footage.

## What is actually verified

**Toolchain and Java — built for real.** `install.sh` was run end to end and `build.sh`
produced `TeamCode-debug.apk` (51 MB) against FTC SDK v12.0, AGP 8.13.2, Gradle 9.1.0,
compileSdk 30. All three classes are present in the APK's dex, and the `Mecanum TeleOp`
registration string with them, so the OpMode will list on the Driver Station. The Java
also compiles clean against the real `RobotCore`/`Hardware` 12.0.0 jars, and every
Limelight method it calls was checked against those jars with `javap`.

**Not verified:** nothing has run on a Control Hub. No robot, no camera, no field.

**Vision — exercised against synthetic frames**, `selftest.py` passes clean: detected
across apparent radii of 10–130 px, which at the real POLLEN diameter is 1.31 m down to
0.10 m; center within 5% of radius, radius error ≤ 4.8%, all four distractor classes
rejected, and a ball fused to a same-hue slab recovered to within 3.2 px and −2.2%
radius. Still unverified on hardware: the HSV bounds and the Hough fallback's cost on
the Limelight's own CPU. `BALL_DIAMETER_M` is now the official figure rather than a
guess, but the tolerance on a moulded ball is real — measure a few.

## Troubleshooting

**`project name '.ftc-sdk' must not start or end with a '.'`** — Gradle 9 refuses to
configure a root project in a dot-directory. This is why the SDK clone is `ftc-sdk/`
and not hidden. Do not rename it.

**`packageDebug` fails once, then succeeds on a retry.** Seen once in three builds, as
`PackageAndroidArtifact$IncrementalSplitterRunnable`, and not reproducible from clean.
Re-run `./build.sh`. If it becomes repeatable, raise `org.gradle.jvmargs` in
`ftc-sdk/gradle.properties` — it ships at `-Xmx1024M`.

**`./build.sh` says `.ftc-env.sh` is missing** — run `./install.sh` first; that file is
generated and holds machine-specific paths, so it is deliberately not tracked.

**Licenses.** If a gradle build fails with an Android package it cannot find, the SDK
licenses were not accepted. Re-run `./install.sh`, which accepts them explicitly — the
failure otherwise surfaces much later and reads like a network problem.

## License

MIT, see `LICENSE`.
