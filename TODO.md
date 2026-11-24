# Team to-do list

Things the robot or the code needs. Items 1–6 come from a code review on 2026-10-01;
items 7–9 were added on 2026-10-19 by the work that split TeleOp and Autonomous by
alliance, added the park, and put the Control Hub's IMU into the auto; item 10 was added
on 2026-11-19 from the Limelight camera-calibration work, and item 11 on 2026-11-23 from
field experience with the shoot-then-drive autos; item 12 on 2026-11-24 after the flywheel
was weighted and a speed droop was noticed on a run-down battery. **These are for the
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

**What is already done — verified 2026-11-19, don't break it.** Every mode that knows its
alliance already filters to its own SCORING cluster: `2a`/`2b` (the filter lives in
`TankTeleOpBase.rangeToHive` / `bearingToHive`), `9red`/`9blue`, `13`/`14`. A red robot
cannot range off a blue or audience tag in any of them.

**What is left.** Three single modes have no alliance to filter with and still ask for
`nearestRangeIn()` — the distance to the closest tag of ANY kind:

- `ShooterOpMode.java` (~line 75) — `6. Shooter (Limelight)`, enabled;
- `MecanumTeleOp.java` (~line 133) — `2. Mecanum TeleOp`, enabled, the practice bot's
  match TeleOp;
- `TankTeleOp.java` (~line 141) — `2. Tank TeleOp (kitbot)`, kept `@Disabled`, so it is
  harmless until someone re-enables it, and then it is a bug.

(`5. Shooter Calibrate` is separate and CORRECT as written: it ranges the single tag you
set on the target stand, so "nearest" is exactly the tag on the goal. Leave it.)

**How to fix — a real decision per mode, not one answer for all three.** The rule that
decides is the codebase's own: the alliance has to be TOLD, never inferred, so a mode with
no alliance slot must not auto-range. Two ways out, and working code exists for both:

1. Give the mode an alliance: split it into a thin red/blue pair exactly like
   `TankTeleOpRed`/`TankTeleOpBlue` — move the body to a base class, add
   `protected abstract HiveGeometry.Cell scoringCell()`, and range on the MEAN of the
   goal cluster's member tags (copy `sumRange`/`nRange` from
   `ForwardOneAimAndShootAutoBase.aimAndShoot()`, which averages the cluster's member
   ranges and skips tags without a 3D pose).
2. Or the opposite: if the mode never needs table-based speed on a match field, call
   `shooter.fallBackToManual()` once at init — the same mechanism a camera-less robot
   already uses — so it never uses the table and never ranges at all. The OpMode keeps
   working as a fixed-speed rig.

**How to check.** With the goal's tags in view at ~60 in and a NEARER audience tag also
in view, the mode's range line must read ~60 in and its asked speed must match what the
table says for ~60 in. With only the audience tag visible: an alliance-split mode shows
no range and refuses; a manual-only one shows MANUAL and the dpad works.

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

## 9. The shooter speed is only measured straight-on

The flywheel was replaced with a heavier one on 2026-11-18. `Shooter.RANGE_VELOCITY_TABLE`
is a flat **1200** ticks/s measured straight-on -- fairly consistent, but not perfect, and
only at one distance. Every auto's `LAUNCH_VELOCITY` plus
`ShooterControls.DEFAULT_MANUAL_VELOCITY` is 1200 too. To turn this into a curve: measure a
second distance in `2a`/`2b` (press dpad until the shots land, read `asked … ticks/s` off
the launcher telemetry) and add a row to the table.

Note the teleop restores the driver's last saved speed from SharedPreferences, so an
already-run robot keeps its old saved value until it is cleared or tuned with the dpad.

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

## 11. Catch a misaligned autonomous start from the tags, BEFORE the HIVE tips and the tags are gone

**What's wrong.** Autos 11/12 and 13/14 assume the robot is staged square on its tile and
pointed where the path expects. Nothing measures that. If staging is off by a few degrees
-- hand-pushed, wheels not square, carpet uneven -- every dead-reckoned leg inherits the
error, and the parking endpoint, a zone only ~23 in wide, misses. The disabled aim-and-park
autos already corrected for one heading error they caused themselves
(`ForwardOneAimAndShootAutoBase.squareUp()`); no ENABLED auto checks the setup angle at all.

**Why the estimate has to happen during shooting.** The HIVE is a seesaw: as the balls land
in the cell we fire at, it tips about 60 deg, the tag clusters rotate with it, and they
leave the camera's view. After that there is no second chance to measure. Whatever heading
record we want must be captured while the tags are still in view -- during spin-up and the
feed, before (or as) the last ball tips the HIVE -- and then survive in the run report for
the parking legs to use.

**Two camera signals are available, and they fail differently -- bench one, then decide:**

1. **Bearing** (the cheap one, already computed elsewhere): the mean `txDeg` of OUR scoring
   cluster. If staging were perfect this reads ~0 at START. It answers "how far left/right
   of the goal am I pointed". It cannot see one case: the tag dead-center but viewed
   obliquely.
2. **Tag-plane rotation** (the idea worth testing): a tag seen straight-on presents its
   plane at the known settled angle; seen from an angle, the image-space rotation of the
   tag carries a residual that tx reports as ~0. The Limelight's fiducial result already
   contains a 3D solve; `LimelightHiveTracker.Tag` carries txDeg/tyDeg/range/tilt/height
   but NOT this in-image skew, so it needs a new field extracted in `toTag()`. If it turns
   out noisy on a real CELL, write that down and use bearing alone -- a documented
   limitation beats a guessed signal.

Gate either signal on SETTLED observations (the `HiveGate` verdict `13/14` already
produces in `waitForHigh()`), so a swinging CELL cannot feed a staging estimate.

**What to do.**

1. **Latch it while the tags are visible.** In `13/14` the hook is `waitForHigh()` (tags in
   view for up to 20 s); in `11/12` the OpMode has no tracker at all yet, so constructing
   one at init is part of this item. Capture once, record into the existing `log.row()` run
   report ("staging=+4.2deg"), and keep it for the park.
2. **Then pick a policy per magnitude** (start with: <= 3 deg just record; larger, act --
   3 deg over a 96 in leg is ~5 in of drift, about a quarter of the zone):
   (a) STRAIGHTEN: rotate by the estimate right after capture, before or during spin-up --
   the drive is idle then anyway; or
   (b) FIX THE PARK: leave the robot alone and fold the correction into the first parking
   turn (90 deg becomes 90+corr) so the endpoint lands despite the crooked start. (a) is
   safer to reason about; (b) is what "pick a better angle for the parking routine" means
   and avoids turning a moving robot while the wheel is spinning up.
3. **Keep it in one place, split by job.** Reading the angle is camera work --
   `LimelightHiveTracker` -- and the policy is field strategy -- the auto. Do not copy the
   loop into each OpMode; two enabled autos sharing one helper is what stopped red and
   blue from diverging.

**How to check.**

1. Bench: stage the robot visibly crooked (~5 deg off the HIVE line), tags in view, and
   confirm the telemetry/report reads ~5. Straight-on staging must read ~0.
2. Tip the CELL by hand (or shoot real balls) and confirm the captured angle SURVIVED in
   the report after the tags are gone -- that is the whole point of the latch.
3. Floor: run `11` with a deliberately crooked start and compare the endpoint to the
   report's angle. Across a ~96 in path, error should track tan(angle) x distance -- 5 deg
   ~= 8 in. If it does not, the angle or the correction leg is wrong.
4. Confirm nothing blocks the run: no tags (camera absent or CELL empty from the previous
   match) must degrade to "staging=unmeasured" and run exactly as today, never hang.

**When it is done:** delete this item in the same commit as any fix.

---

## 12. Measure the launcher's PIDF gains — the wheel has been weighted, and the numbers are still goBILDA's

**What's wrong.** The launcher's speed loop was copied from goBILDA's BIOBUZZ example and
has never been measured on this robot: `PIDF_P = 40.0`, `PIDF_F = 12.5`, `I = D = 0`
(`Shooter.java` lines 62-65, applied at 135-137). Since then the flywheel has had weight
added, which changes the job: a heavier wheel dips LESS when a ball takes its energy but
recovers SLOWER, so the response of the loop needs re-measuring against inertia that did
not exist when the numbers were written. The symptom that started this item: `actual`
sagging below `asked` as the battery ran down.

**FIRST, for one session, tell the two droop causes apart. Only one is tuning:**

| what the telemetry shows | what it is | what fixes it |
|---|---|---|
| `actual` dips on ball contact and recovers slowly or softly, on a HEALTHY battery | loop bandwidth -- the gains | tune F and P below |
| `actual` sits below `asked` only while the battery is low | NOT necessarily saturation. If pushing the power up BY HAND makes it hold -- the drivers found exactly this -- the pack still has headroom, and the missing piece is the I term (step 7). If even that cannot hold the speed, it is true saturation: fresh battery or a lower target |

The calibrate OpMode's warning line ("WHEEL CANNOT REACH asked", at `actual < 0.95 x
asked`) is the existing detector for the second case. No tuning session is valid while
it is lit.

**How to tune (one knob at a time; re-enable `5. Shooter Calibrate` for the session,
re-disable it after):**

1. **Baseline, nothing changed.** Fresh battery, wheel at a known speed, feed five balls
   one at a time. From the "wheel ... asked ... actual" line, write down per ball: dip
   depth and the time to return to `asked`. This table of numbers is what every change
   is judged against.
2. **Drag check on the added weight.** Power OFF, spin the wheel by hand, time the
   coast-down. Inertia alone should make dips SMALLER and recoveries slower -- so if the
   weighted wheel now dips MORE than the unweighted one did, the weight added friction
   (bearing drag, an off-balance wheel), and that is a mechanical fix, not a gain.
3. **F is the steady-state knob.** If `actual` parks a little below `asked` at rest on a
   healthy battery, the P term is carrying what feed-forward should. Raise F in 10-20%
   steps.
4. **P is the recovery knob.** Only if per-ball recovery is slower than the feed pace
   needs: raise P in ~50% steps, re-run the baseline, compare. Two signs of too much P:
   overshoot/ringing at spin-up (the wheel passes its target and oscillates), and twitchy
   `actual` between balls. Back off to the last quiet setting.
5. **Keep D at 0, and I at 0 until step 7 says otherwise.** They are 0 deliberately;
   adding unknowns mid-session is how tuning becomes debugging.
6. **Write it down in the standard now set:** beside the constants go the new values,
   the battery voltage, the flywheel weight, and the date -- the same culture the
   calibration table adopted with its per-row voltage comments. An unlabelled number is
   exactly the failure that low-battery calibration taught us about.

Also worth writing down while the wheel is off the robot's schedule: the weighted wheel
takes LONGER to spin up. If the autos' `SPIN_UP_MAX_WAIT_MS` windows or the feed pace
start looking tight in the run reports, that shows up here as feeds beginning on a wheel
that never reported READY.

**How to check.** Same distance as a calibrated table row, several balls: shots must land,
and `actual` must return to `asked` between balls. Then repeat with the tired pack: some
sag under load is physics, but a *steady* offset below `asked` at a HEALTHY battery is a
gain problem -- and the reverse split has to hold too, or the distinction in the table
above was wrong and the item is not done.

7. **The automatable version of the drivers' power bump: the I term.** Raising the power
   by hand on a low battery, shot after shot, is a job for a controller, and the I term
   is exactly that job: it raises the POWER continuously while `actual` sits below
   `asked`, without touching the target, so the target stays the true table speed. Set
   `PIDF_I` (0.0 today, `Shooter.java` line 63) small -- a first step is about
   `P/1000` (0.04), then step by 3x, judged only from the baseline feeds. Pass test: on
   the TIRED pack, `actual` must close to `asked` by itself, between balls, with no
   driver input, and must not overshoot badly after a deep dip. If I winds up and
   overshoots, back it off; if it cannot be made stable at all, the fallback is
   voltage-compensated feed-forward: scale the F feed by `NOMINAL_V / getBatteryVoltage()`
   -- the voltage readout already exists in `Shooter` -- but only with a bench
   measurement behind it, for the same reason as the table rows.

8. **Option B: do the same fix in code instead of firmware** -- the drivers' manual
   routine is already an outer integral loop, so it can be a naive one:
   every ~0.5 s, if `actual` has been below `asked`, add a small step (1-2% of the target)
   to a `droopCompensation` offset and pass `asked + offset` to
   `Shooter.setVelocity`; if `actual` climbs above `asked`, take a step back down.
   Three rules make it safe:
   (a) CAP the offset (e.g. +-100 ticks/s) and RESET it when the base target changes,
   the feed ends, or the launcher disarms -- a software integrator winds up forever
   unless we own the unwind;
   (b) SHOW the offset separately in telemetry, next to (not inside) the asked line;
   (c) a recorded calibration point must carry the BASE speed, never the base plus
   offset -- otherwise battery droop bakes itself into the table, which is exactly how
   the low-battery rows in item 9 got their error. Option A is invisible but is one
   constant; Option B is visible, loggable in the run report, and can be gated to engage
   only below a chosen battery voltage -- pick by which failure you would rather debug
   at a venue.

**When it is done:** delete this item in the same commit as any fix.

---

## Small cleanups

- `ForwardAndTurnAuto.java`: the comment "Hold the final position" isn't quite true —
  `stop()` just brakes, nothing actively holds.
