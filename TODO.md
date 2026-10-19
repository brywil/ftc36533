# Team to-do list

Things the robot or the code needs. Items 1–6 come from a code review on 2026-10-01;
items 7–9 were added on 2026-10-19 by the work that split TeleOp and Autonomous by
alliance, added the park, and put the Control Hub's IMU into the auto. **These are for the
team to do** — each one says what is wrong, why it matters, where to look, and how to check
your fix. Do them in order; the first one is a safety issue.

When you finish one, delete it from this list in the same commit as the fix.

---

## 1. Plug in the drive motor encoder cables (do this before running Autonomous)

**What's wrong.** `8. Auto: 2 Squares, Turn 90, Launch` (`ForwardAndTurnAuto.java`) and the
`9red`/`9blue` autos measure how far the robot has driven with the **drive motors'
encoders**. Right now only the launcher has its encoder cable plugged in, and `WIRING.md`
says the other motors don't need one.

**Why it matters.** With no encoder cable, the motor's position always reads 0, so the
robot never "arrives". It keeps driving at half power until the timeout — several metres
across the field — and then spins for several more seconds. That is a runaway robot, not a
missed target.

**What to do.**
1. The goBILDA drive motors already have encoders built in; only the cable to the Control
   Hub is missing. Plug an encoder cable from each drive motor (`left_drive`,
   `right_drive`) into the Control Hub's encoder port **with the same number as that
   motor's power port**.
2. Fix `WIRING.md`: the line "The other three motors do not need encoders" (line ~76) is
   now wrong. Say the drive motors need encoders for Autonomous.
3. Check it works **with the robot on blocks** (wheels off the ground): run the auto.
   The Driver Station shows `left` and `right` as "ticks moved / ticks wanted". Both
   numbers should climb and the wheels should stop on their own when they reach the
   target. If a side stays at 0, that side's encoder cable is missing or in the wrong
   port.

---

## 2. Make Autonomous stop if the encoders aren't working — CODE DONE, NOT YET TESTED ON THE ROBOT

**What's wrong.** Even after #1, a cable can come loose. `runMove()` in
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

## 3. Measure the three numbers Autonomous depends on — MEASURED, CODE UPDATED, NOT YET VERIFIED ON THE FLOOR

Measured 2026-10-07 and written into `ForwardAndTurnAuto.java`:

- `TICKS_PER_WHEEL_REV = 537.7` is **correct** — it is the **19.2:1** gearbox
  (28 counts/motor-turn x 19.2), not the 5.23:1 the old comment claimed. Confirmed by
  hand: 2683 ticks over 5 wheel turns = 536.6. Only the comment was wrong; it is fixed.
- `WHEEL_DIAMETER_IN = 3.75` (tape across the tread; was 4.0).
- `TRACK_WIDTH_IN = 15.75` (centre of left wheel to centre of right; was 12.0).

**STILL TO DO — verify on the floor.** The file's header explains the one-tile push test:
push the robot exactly one tile and compare the encoder count with what the code expects.
Do that (and the on-blocks auto run) before trusting a score, then delete this item.

Note the park now leans on two more geometry numbers — `PARK_DISTANCE_IN` and
`TRACK_WIDTH_IN` again for the 90 deg turn — so a track-width error shows up twice.

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

## 5. Shooter: aim at the goal's tag, not just the nearest tag — PARTLY DONE

Also only for the Limelight OpModes.

**What's wrong.** The shooter asks for `nearestRangeIn()` — the distance to the closest
AprilTag of **any** kind. If another tag on the field is closer than the goal, the robot
shoots for the wrong distance.

**Partly done.** `9red`/`9blue` no longer do this: they filter to their own alliance's
SCORING cluster (`scoringCell()`), so a red auto cannot range off a blue or audience tag.
The standalone `6. Shooter` OpMode still uses `nearestRangeIn()`.

**What to do.** Look up the goal's tag ID in the game manual and use the tracker's
`tagWithId(id)` method (in `LimelightHiveTracker.java`) instead of the nearest one.

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

## Small cleanups

- `ForwardAndTurnAuto.java`: the comment "Hold the final position" isn't quite true —
  `stop()` just brakes, nothing actively holds.
