# Team to-do list

Things the robot or the code needs, found in a code review on 2026-10-01. **These are
for the team to do** — each one says what is wrong, why it matters, where to look, and
how to check your fix. Do them in order; the first one is a safety issue.

When you finish one, delete it from this list in the same commit as the fix.

---

## 1. Plug in the drive motor encoder cables (do this before running Autonomous)

**What's wrong.** `8. Auto: 2 Squares Forward, Turn 72` (`ForwardAndTurnAuto.java`)
measures how far the robot has driven with the **drive motors' encoders**. Right now only
the launcher has its encoder cable plugged in, and `WIRING.md` says the other motors
don't need one.

**Why it matters.** With no encoder cable, the motor's position always reads 0, so the
robot never "arrives". It keeps driving at half power until the 8-second timeout — several
metres across the field — and then spins for another 5 seconds. That is a runaway robot,
not a missed target.

**What to do.**
1. The goBILDA drive motors already have encoders built in; only the cable to the Control
   Hub is missing. Plug an encoder cable from each drive motor (`left_drive`,
   `right_drive`) into the Control Hub's encoder port **with the same number as that
   motor's power port**.
2. Fix `WIRING.md`: the line "The other three motors do not need encoders" is now wrong.
   Say the drive motors need encoders for Autonomous.
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

## 3. Measure the three numbers Autonomous depends on

In `ForwardAndTurnAuto.java`, two constants disagree with their own comments:

- `TICKS_PER_WHEEL_REV = 537.7`. The comment says that's a 5203 motor at 5.23:1, but
  goBILDA motors give 28 counts per motor turn, and 537.7 ÷ 28 = 19.2 — so 537.7 is the
  **19.2:1** gearbox. Look at the label on your drive motors and use the right number
  (goBILDA's product page lists it). If the robot has the 5.2:1 gearbox, every distance is
  about 3.7× off.
- `WHEEL_DIAMETER_IN = 4.0`, but the comment says the wheel is 3.78 in. Measure your
  wheels with a tape.
- `TRACK_WIDTH_IN = 12.0` — measure it too (centre of left wheel to centre of right).

**How to check.** The file's header explains the one-tile push test: push the robot
exactly one tile and compare the encoder count with what the code expects.

---

## 4. Shooter: don't shoot before it's calibrated

Only matters once the Limelight shooter OpModes are turned back on (they're `@Disabled`
right now).

**What's wrong.** `Shooter.java` says it "fails closed" — it should refuse to fire until
the distance→speed table has been measured. But it only checks that the table has at
least 2 rows, and the file ships with 4 **made-up** rows, so it always fires.

**What to do.** Add something like `CALIBRATED = false` that `rangeToVelocity()` checks,
and set it to `true` only when you paste in real numbers from `5. Shooter Calibrate`.

---

## 5. Shooter: aim at the goal's tag, not just the nearest tag

Also only for the Limelight OpModes.

**What's wrong.** The shooter asks for `nearestRangeIn()` — the distance to the closest
AprilTag of **any** kind. If another tag on the field is closer than the goal, the robot
shoots for the wrong distance.

**What to do.** Look up the goal's tag ID in the game manual and use the tracker's
`tagWithId(id)` method (in `LimelightHiveTracker.java`) instead of the nearest
one.

---

## 6. Shooter: don't stop the wheel when the camera blinks

Also only for the Limelight OpModes.

**What's wrong.** In AUTO mode, if the Limelight misses the tag for a single frame, the
shooter asks for speed 0, the wheel slows, and you have to wait for it to spin up again.

**What to do.** Remember the last good distance and keep using it for a short time (try
300 ms) before giving up.

---

## Small cleanups

- `ForwardAndTurnAuto.java`: the comment "Hold the final position" isn't quite true —
  `stop()` just brakes, nothing actively holds.
- `AutoAim.java`: `clamp()` has the line `if (mag > max) mag = max;` twice.
