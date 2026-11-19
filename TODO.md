# Team to-do list

Things the robot or the code needs. Items 1–6 come from a code review on 2026-10-01;
items 7–9 were added on 2026-10-19 by the work that split TeleOp and Autonomous by
alliance, added the park, and put the Control Hub's IMU into the auto; item 10 was added
on 2026-11-19 from the Limelight camera-calibration work. **These are for the
team to do** — each one says what is wrong, why it matters, where to look, and how to check
your fix. The numbers are still the rough priority order; item 1, the safety issue, is
done, and the rest keep their numbers because code comments reference them by number
(see "TODO #6" in ForwardOneAimAndShootAutoBase.java).

Items 1 (drive encoder cables + WIRING.md) and 3 (the measured drive geometry) are DONE
and were removed on 2026-11-19.

When you finish one, delete it from this list in the same commit as the fix.

---

## 2. Make Autonomous stop if the encoders aren't working — CODE DONE, NOT YET TESTED ON THE ROBOT

**What's wrong.** Even with the drive encoder cables correctly plugged in, a cable can
come loose. `runMove()` in
`ForwardAndTurnAuto.java` only gives up when the timeout runs out.

**What's done.** `runMove()` now checks each side on its own: if a side was asked to
move but has counted 0 ticks after `ENCODER_STARTUP_GRACE_MS` (500 ms), it stops the
motors and returns "STOPPED: ... drive encoder not reading -- check the cables" for the
Driver Station. Per-side, because one loose cable lets the other side run on (the robot
curves on the straight, or spins until timeout on the turn).

**STILL TO DO — verify on the robot.** This is written but has never run on hardware.
Unplug one encoder cable, put the robot on blocks (wheels off the ground), run the auto.
It should stop within about a second and show the message. If it does, delete this item
from the list in the same commit as any fix.

---

## 4. Shooter: don't shoot before it's calibrated

Only matters once the Limelight shooter OpModes are turned back on (`6. Shooter` is still
`@Disabled`).

**What's wrong.** `Shooter.java` says it "fails closed" — it should refuse to fire until
the distance→speed table has been measured. But it only checks that the table has at least
`MIN_POINTS` rows, and two identical rows satisfy that, so it always fires.

**What to do.** Add something like `CALIBRATED = false` that `rangeToVelocity()` checks,
and set it to `true` only when you paste in real numbers from `5. Shooter Calibrate`.

---

## 5. Shooter: range the goal's tags, not the nearest tag — THE LAST PIECE

**What's wrong.** The standalone `6. Shooter` OpMode still asks for `nearestRangeIn()` —
the distance to the closest AprilTag of **any** kind. If an audience tag or the opponent's
cluster is closer than the goal, it ranges for the wrong distance and the speed comes
from the wrong table row, so the shot is wrong. The autos already fixed this half:
`9red`/`9blue` filter to their own alliance's SCORING cluster. What is left is exactly
one loop:

- `ShooterOpMode.java`, lines 74–76: `tracker.nearestRangeIn()` feeds `shooter.update(...)`.

**What to do.** Two sizes of fix; there is working code to copy for both:

1. Smallest: pin one tag ID (this was the code review's original suggestion). Look up the
   goal's tag ID in the game manual and use the tracker's `tagWithId(id)` (in
   `LimelightHiveTracker.java`) instead of nearest-of-everything. The catch: the goal's
   tag ID is alliance-specific, so one OpMode cannot know it -- the alliance has to be
   TOLD, which is exactly why `2a`/`2b` and `9red`/`9blue` exist.
2. Recommended, and the pattern everything else already follows: split `6. Shooter` into
   a thin red/blue pair like `TankTeleOpRed`/`TankTeleOpBlue`. Move the body into a
   `ShooterOpModeBase`, add `protected abstract HiveGeometry.Cell scoringCell()`, and
   range on the MEAN of whichever member tags of that cluster are in view — the red
   scoring cluster is ids 30-33, blue is 42-45 (`HiveGeometry.Cell`). Copy the
   filter-and-mean from `ForwardOneAimAndShootAutoBase.aimAndShoot()` (the part that
   builds `sumRange`/`nRange`), which already averages member ranges, skips tags
   without a 3D pose, and falls back gracefully. A red robot then cannot range off a
   blue or audience tag, and the alliance becomes a visible Driver Station choice.

**How to check.** Bench, camera in its fiducial pipeline, launcher armed (hands clear of
the muzzle). Put OUR goal's tags in view at, say, 60 in, and nearer in view put an
AUDIENCE tag. The telemetry's range must read the ~60 in goal distance, not the near
tag, and the asked speed must match the table row that ~60 in interpolates to. Also
verify with only the audience tag in view: the OpMode should show NO RANGE rather than
fire for an audience distance.

**When it is done:** delete this item in the same commit as any fix.

---

## 6. Shooter: don't stop the wheel when the camera blinks

Also only for the Limelight OpModes.

**What's wrong.** In AUTO mode, if the Limelight misses the tag for a single frame, the
shooter asks for speed 0, the wheel slows, and you have to wait for it to spin up again.

**What to do.** Remember the last good distance and keep using it for a short time (try
300 ms) before giving up. The auto already does this with its own `RANGE_HOLD_MS`; the fix
belongs in `ShooterControls` so every mode gets it.

---

## 7. Put the IMU in the Robot Configuration — the park now depends on it

**What's wrong.** `9red`/`9blue` now use the Control Hub's **built-in IMU** to square the
robot up before parking. The code asks for it as `imu` and **degrades quietly if it is
missing**: `updateHeading()` falls back to counting wheels. The kitbot's Robot
Configuration has never had an IMU, because four documents said one was not needed.

**Why it matters.** The park drives 96 in along a wall after an open-loop aim turn. With
the gyro that leg starts square to the wall; without it the heading comes from wheels that
scrub on carpet, and the robot can miss a zone only 23 in wide.

**What to do.**
1. On the Driver Station, add **IMU** to the kitbot's Robot Configuration, named `imu`.
2. Confirm the orientation matches the robot: the hub sits upright with its **logo facing
   the robot's right** and its **USB port up**, which is what the auto passes to
   `RevHubOrientationOnRobot` (`LogoFacingDirection.RIGHT`, `UsbFacingDirection.UP`). If the
   hub is ever remounted, those two values must change with it.
3. Check it on blocks: run `9red` and, **before pressing START, spin the robot by hand**.
   The `heading (spin me by hand)` line must change as it rotates, and must be **positive
   when you turn it clockwise**. A heading that barely moves means the wrong axis; a
   heading that goes negative means the wrong sign. Either way the park would square up to
   a nonsense angle, so fix it before trusting the park.
4. Then fix the documents that still say the opposite: `README.md` (~line 165, "No IMU is
   needed or used"), `GETTING_STARTED.md` (~line 221) and the header of
   `ForwardAndTurnAuto.java` (~line 9, "there is no IMU on the kitbot").

---

## 8. Run the new Autonomous on the floor

**What's wrong.** `9red`/`9blue` now drive **2 squares** (48 in), aim, feed for 10 s, then
park: back to the wall, turn 90° right, drive 96 in along it into the LOADING ZONE. None of
that has run on a field, and the OpModes were renamed (`1 Square` → `2 Squares, Aim,
Shoot, Park`), so check you are picking the right one.

**Why it matters.** PARK is 5 match points assessed at the end of AUTO (BIOBUZZ 10.5.4),
and the rule is only "at least partially in the LOADING ZONE". Several numbers behind it are
estimates: `PARK_DISTANCE_IN = 96` is a hand measurement, `PARK_WALL_CLEARANCE_IN = 4` is a
judgement call to try to score LEAVE (3) and PARK (5) together, and the whole run has to
fit in 30 s.

**How to check.** On blocks first — wheels off the ground — then on the floor. Read the run
report off the Driver Station: `move`, `shot`, `park`. The `park` line names the leg that
failed if one does. If it is still feeding when the 30 s ends, cut `FEED_DURATION_MS` — not
the park.

---

## 9. The shooter speed is a guess

`Shooter.RANGE_VELOCITY_TABLE` is **1275** ticks/s, which is one dpad step below the 1300
that was measured on 2026-10-15 and then overshot. **Nobody has watched a ball land on
1275.** To replace it with a measured number: in `2a`/`2b`, press dpad-down until the shots
land, read `asked … ticks/s` off the launcher telemetry, and paste that into the table.

Related: `ForwardAndTurnAuto.LAUNCH_VELOCITY` (the `8. Auto`) is a separate **1300** that
did not get this change. If that auto overshoots too, lower it there as well.

Do not go below `Shooter.MIN_LAUNCH_VELOCITY` (800): below that the wheel can never measure
as spun up, the windmill never opens, and nothing is fired at all.

---

## 10. Save the Limelight's tuned exposure/gain on the Control Hub, and re-assert it whenever a mode starts

**What's wrong.** `4b. Limelight Auto Calibrate` sweeps exposure and gain over the camera's
REST API and applies the winner to pipeline 1 with `POST /upload-pipeline`, which per
Limelight's own API docs overwrites the pipeline **on disk** — so the setting normally
survives a camera reboot. But the result lives only inside the camera: nothing on the robot
records it. If the camera's pipeline is later replaced from its web page, the flash is
wiped, or a slider gets nudged, the next match runs on whatever the camera happens to hold,
and nothing on the hub can say what that was.

**The design (agreed 2026-11-19).** Make the ROBOT the keeper of the tuned values, and make
applying them automatic — so it never depends on anyone reading the hub screen, which is
already too busy to be read.

1. **`4b` saves the winner.** When it applies the winner (in
   `LimelightAutoCalibrateOpMode.java`, right after `winnerApplied = true`), write a small
   JSON file — one line is plenty:

   ```json
   { "exposure": 400, "lcgain": 50, "detect": 87, "recorded": "2026-11-19 14:22" }
   ```

   `hardwareMap.appContext.getExternalFilesDir(null)` gives an app-private folder: no
   permissions needed, survives a hub reboot, readable over `adb`. Also log the same
   numbers with `RobotLog`, plus the original settings and the P/. grid, so the record for
   whoever debugs at a venue is in the RC log, not on a screen.

2. **Every camera OpMode re-asserts it.** All camera-using OpModes build a
   `LimelightHiveTracker` and call `start()`, so the apply belongs there — one place, not
   seven. In `LimelightHiveTracker.start()`, AFTER `pipelineSwitch(...)` (the update must
   land on the pipeline that is running), read the file and POST
   `{"exposure": …, "lcgain": …}` to `/update-pipeline`.

   PATCH, never upload: `/update-pipeline` takes partial settings, and without `flush=1`
   it writes nothing to the camera's disk — the re-assert is a runtime patch onto
   whatever the camera has, not a second calibration.

   The HTTP helper the sweep uses lives inside `LimelightAutoCalibrateOpMode.java` as a
   private static. Move it out into one small shared class (e.g. `LimelightSettings`) so
   the save and the apply own a single copy of it.

3. **Missing file or silent camera never blocks anything.** No file → apply nothing; the
   camera keeps its own values. Camera not answering → skip, `RobotLog` it, continue; use
   a short connect timeout (~300 ms) so INIT never feels hung. The camera is already
   optional everywhere else; this must be too.

4. **NO NEW TELEMETRY.** The control hub screen is tiny and already carries more than
   anyone reads — this feature gets ZERO new lines on it. If anything, replace the
   existing `limelight: OK/MISSING` line in the OpModes that have it with
   `limelight: 400/50 (tuned)` — same line, strictly more information. The file plus the
   `RobotLog` lines are the whole record.

**How to check.**

1. Run `4b` to completion on the bench. Read the hub's JSON file back over `adb` and
   confirm it holds the winner, the detection rate and a date.
2. Insurance test, without touching the robot: on the camera's web page, set exposure to
   something silly. Select any camera OpMode and START, then STOP. The web page must
   show the tuned values put back — with no other action than selecting the mode.
3. Reboot the camera and re-check `GET /pipeline-atindex?index=1` in a browser. The docs
   say `4b` wrote the winner to the camera's disk and the tracker patch does not write
   to disk, so rebooting must still show them. If they are gone, write down what your
   camera's firmware actually does — that finding is worth more than the feature.
4. Unplug the camera and run a drive OpMode: the robot must still drive, and INIT must
   not hang or crash for a missing camera or a missing file.

**When it is done:** delete this item in the same commit as any fix.

---

## Small cleanups

- `ForwardAndTurnAuto.java`: the comment "Hold the final position" isn't quite true —
  `stop()` just brakes, nothing actively holds.
