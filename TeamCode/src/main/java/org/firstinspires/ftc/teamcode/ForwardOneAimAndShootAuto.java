package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;
import java.util.Locale;

/**
 * Drive one floor tile forward, aim at the nearest AprilTag, then shoot at it.
 *
 * The three pieces this stitches together, each of which already exists and works
 * on its own:
 *
 *   1. a dead-reckoned one-tile move -- {@link ForwardAndTurnAuto}'s geometry and
 *      the same per-side encoder check;
 *   2. aiming -- {@link AutoAim} turns the tag's bearing to zero, and the robot
 *      turns with it while driving nothing else;
 *   3. the shot -- {@link ShooterControls} picks the launcher speed from the tag
 *      range and refuses to feed until the wheel is actually up to speed.
 *
 * Aim and spin-up happen at the same time: the launcher is asked for a speed on
 * the first frame a tag is seen, and the wheel comes up while the robot is still
 * finishing its turn. Feeding waits for BOTH "aimed" (bearing inside AutoAim's
 * deadband for SETTLE_MS) and "spun up" (a real encoder measurement).
 *
 * WHICH ROBOT: the goBILDA StarterBot / kitbot. The drive geometry is reused from
 * {@link ForwardAndTurnAuto} so the one-tile calibration lives in one file. The
 * mecanum practice bot would need its own version (and its own track width).
 *
 * ---------------------------------------------------------------------------
 * THE SPEED IS ONE MEASURED VALUE, NOT A CURVE. Shooter.RANGE_VELOCITY_TABLE holds a
 * single speed -- 1300 ticks/s, tuned by hand in "2. Tank TeleOp" on 2026-10-15 --
 * used flat at every distance, so the camera's range does not change the shot yet.
 * Find a speed for a second distance, add the row, and this becomes distance-aware.
 * Until then treat a shot from an untuned distance as an experiment, not a score.
 * ---------------------------------------------------------------------------
 *
 * SAFETY:
 *   - Wheels off the ground (on blocks) for the FIRST run -- this aims and spins
 *     the launcher, and blocks let you watch both without throwing anything.
 *   - The launcher spins up as soon as a tag is in view, even before the robot is
 *     aimed. Keep hands and faces away from the muzzle the whole time.
 *   - If no tag is in view it searches: it spins slowly in place for up to one full
 *     rotation. Keep clear of the wheels. Once a tag HAS been seen, losing it stops
 *     the turn rather than hunting (see {@link AutoAim}).
 *   - If the tag never settles, or the wheel never reaches speed, it stops on a
 *     timeout instead of firing a guess.
 */
@Autonomous(name = "9. Auto: 1 Square, Aim & Shoot", group = "Auto")
public class ForwardOneAimAndShootAuto extends LinearOpMode {

    // --- Field geometry -----------------------------------------------------

    /** How many floor tiles to drive forward before aiming. The request was one. */
    public static final double SQUARES = 1.0;

    // --- Motion -------------------------------------------------------------

    /** Forward power, matching the other auto. Moderate so the wheels do not slip. */
    public static final double DRIVE_POWER = 0.5;

    /** How close to the target, in ticks, counts as "arrived". */
    public static final int TOLERANCE_TICKS = 15;

    /** Give up after this long with nothing reached, rather than driving forever. */
    public static final long FORWARD_TIMEOUT_MS = 6000;

    /**
     * If neither side's encoder has counted a tick by this long into the move, treat
     * the encoders as dead (usually an unplugged cable or the wrong port) and stop.
     */
    public static final long ENCODER_STARTUP_GRACE_MS = 500;

    // --- Camera ------------------------------------------------------------

    /** Must match the Robot Configuration name of the Limelight. */
    private static final String LIMELIGHT_NAME = "limelight";

    /** The fiducial (AprilTag) pipeline configured on the camera. */
    private static final int FIDUCIAL_PIPELINE = 1;

    // --- Search -------------------------------------------------------------

    /**
     * How to spin while looking for a tag that is not in view. Positive is clockwise,
     * matching the drivebase and AutoAim. One full turn in either direction sees the
     * whole field, so the direction only decides which way it sweeps first.
     */
    public static final double SEARCH_SIGN = 1.0;

    /**
     * Search turn power. Kept below AutoAim.MAX_TURN so the aim loop, which takes
     * over the instant a tag appears, is always turning no faster than the search
     * was. Raise it to search quicker; if the robot sweeps past tags without seeing
     * them, motion blur is the likely reason, so bring it back down.
     */
    public static final double SEARCH_TURN_POWER = 0.4;

    /**
     * How far to search, in degrees of rotation. A little over one full turn so a
     * tag is not missed to wheel slip. There is no IMU, so this is measured from the
     * drive encoders against the same track width the turn uses.
     */
    public static final double SEARCH_SPAN_DEG = 400.0;

    // --- Aiming and shooting ------------------------------------------------

    /**
     * Give up if a tag that IS in view never settles inside AutoAim's deadband within
     * this long, or never reaches launcher speed. Timed from the moment a tag is first
     * seen, so it does not count the search time. A tag that is never seen at all ends
     * the search on its own, at SEARCH_SPAN_DEG.
     */
    public static final long AIM_TIMEOUT_MS = 6000;

    /** How long to keep feeding once the shot is ready. 15 s after settled. */
    public static final long FEED_DURATION_MS = 15000;

    /**
     * How far off centre the aim may be and still fire, in degrees. This is the gate
     * that replaces AutoAim's strict "settled" (inside its 1.5 deg deadband for
     * 250 ms): the tag set is a little jumpy, and a shot that is a few degrees off is
     * good enough. Raise it to fire sooner, lower it to shoot straighter. The wheel
     * still has to be up to speed either way.
     */
    public static final double FIRE_TOLERANCE_DEG = 4.0;

    /**
     * Keep using the last good range for this long when the tag blinks out for a
     * frame or two. Without it a single missed frame asks for speed 0, the wheel
     * slows, and the shot has to spin back up. (The canonical fix belongs in
     * {@link ShooterControls} -- see TODO #6 -- this is the auto-sized version.)
     */
    public static final long RANGE_HOLD_MS = 300;

    /**
     * How many recent camera frames the aim bearing and range are averaged over.
     * The set of tags the camera reports changes frame to frame -- now one member,
     * now two -- so a single frame's group mean jumps (we saw 0..7 deg). A short
     * rolling window over time smooths those jumps into the middle, instead of
     * steering the robot at whichever subset happened to be seen that frame.
     */
    public static final int AIM_SMOOTH_FRAMES = 5;

    /** Drop the smoothing window after this long with no tag, so stale angles cannot bias re-acquisition. */
    public static final long AIM_SMOOTH_RESET_MS = 400;

    /** The status string a move returns when it reached its target. */
    private static final String REACHED = "reached";

    private TankDrivebase drive;
    private LimelightHiveTracker tracker;
    private ShooterControls shooter;

    /** Encoder ticks per inch of wheel travel, from ForwardAndTurnAuto's geometry. */
    private double ticksPerInch;

    // Latched for the end-of-run report: how many frames had a tag, and the last one
    // seen. A single detection would otherwise flash past in the live telemetry.
    private int tagsSeenFrames = 0;
    private String lastTagSeen = null;

    @Override
    public void runOpMode() {
        drive = new TankDrivebase(hardwareMap);
        tracker = new LimelightHiveTracker(hardwareMap, LIMELIGHT_NAME, FIDUCIAL_PIPELINE);
        shooter = new ShooterControls(new Shooter(hardwareMap));

        // One tile expressed in the drive motors' own ticks, using the geometry
        // already measured and recorded in ForwardAndTurnAuto.
        ticksPerInch = ForwardAndTurnAuto.TICKS_PER_WHEEL_REV
                / (Math.PI * ForwardAndTurnAuto.WHEEL_DIAMETER_IN);
        int forwardTicks = (int) Math.round(SQUARES * ForwardAndTurnAuto.TILE_IN * ticksPerInch);

        telemetry.addLine("Auto: 1 square forward, aim at the middle of the tags, shoot.");
        telemetry.addLine("WHEELS OFF THE GROUND for the first run.");
        telemetry.addLine("Launcher spins as soon as a tag is in view -- keep clear.");
        telemetry.addLine();
        telemetry.addData("forward", "%d ticks (%.1f in)",
                forwardTicks, SQUARES * ForwardAndTurnAuto.TILE_IN);
        telemetry.addData("limelight", tracker.hasLimelight() ? "OK" : "MISSING from config");
        telemetry.addData("launcher", shooter.shooter().hasLauncher() ? "OK" : "MISSING");
        telemetry.addData("windmill", shooter.shooter().hasWindmill() ? "OK" : "MISSING");
        telemetry.addData("table", Shooter.RANGE_VELOCITY_TABLE.length
                + " points -- one speed (%.0f) for every distance",
                Shooter.RANGE_VELOCITY_TABLE[0][1]);
        telemetry.addLine();
        telemetry.addLine("The speed is measured for one distance, not a curve yet.");
        telemetry.addLine("Add a row for a second distance to make it distance-aware.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Start the camera before the move so it is warmed up and streaming by the
        // time the robot stops; the first frames after a cold start are dropped.
        tracker.start();

        String moveResult = driveForward("forward " + format(SQUARES) + " square", forwardTicks);
        drive.stop();

        String shotResult = aimAndShoot();

        // Everything off, whatever happened.
        drive.stop();
        shooter.stop();
        tracker.close();

        telemetry.addData("move", moveResult);
        telemetry.addData("shot", shotResult);
        telemetry.addData("tags seen", tagsSeenFrames == 0
                ? "NEVER -- the camera saw no AprilTag this run"
                : tagsSeenFrames + " frame(s); last: " + lastTagSeen);
        telemetry.update();

        // Leave the result up for a moment so it can be read on the Driver Station.
        sleep(2000);
    }

    /**
     * Find a tag (spin in place if none is in view), center the nearest one, spin
     * the launcher to the range's speed, and feed once aimed AND up to speed.
     * Returns a status string for the Driver Station describing how it ended.
     *
     * One loop does all of it because the jobs are coupled: the same tag gives both
     * the bearing (aim) and the range (launcher speed), and feeding must wait on
     * both. Splitting them into phases would mean re-reading the tag with a
     * different frame each time.
     *
     * SEARCH ONLY RUNS BEFORE THE FIRST TAG. Once one has been seen, a dropped tag
     * means "no turn" (AutoAim's rule), not another spin: turning mid-shot throws
     * the aim off, and a tag that was just in view is more likely blinking than gone.
     * The search is bounded by the encoders to one rotation, so a field with no tag
     * in range stops instead of spinning forever.
     */
    private String aimAndShoot() {
        AutoAim aim = new AutoAim();
        aim.reset();                 // no stale error before the first turn
        shooter.setArmed(true);
        shooter.setManual(false);    // AUTO: speed from the measured table and the range

        // One full search sweep, in encoder ticks, from the same wheel geometry the
        // turn uses: each side rolls the arc swept by a point at the track radius.
        double searchTicks = (SEARCH_SPAN_DEG / 360.0) * Math.PI
                * ForwardAndTurnAuto.TRACK_WIDTH_IN * ticksPerInch;

        Double lastRangeIn = null;
        long lastRangeMs = 0L;

        // Where the encoders started, so the search can tell how far it has swept.
        int searchLeftStart = drive.leftTicks();
        int searchRightStart = drive.rightTicks();

        long feedStart = 0L;
        long aimStart = 0L;          // set the first time a tag is seen
        boolean everAimed = false;

        // Rolling windows over the last few frames, so the aim does not chase the
        // per-frame change in which tags were detected (see AIM_SMOOTH_FRAMES).
        double[] bearingWin = new double[AIM_SMOOTH_FRAMES];
        int bearingCount = 0, bearingNext = 0;
        double[] rangeWin = new double[AIM_SMOOTH_FRAMES];
        int rangeCount = 0, rangeNext = 0;
        long lastTagFrameMs = 0L;

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            // Aim at the MIDDLE of the tags in view, not at one of them. Aiming at
            // the nearest tag lets the choice flip between adjacent members as their
            // ranges cross, so the bearing jumps between them and the robot dithers
            // and never settles. Averaging the group's bearings cancels that, and the
            // per-tag camera noise with it. The range is the group's mean distance.
            List<LimelightHiveTracker.Tag> tags = tracker.recognize();
            double sumTx = 0.0, sumRange = 0.0;
            int nTx = 0, nRange = 0;
            for (LimelightHiveTracker.Tag t : tags) {
                sumTx += t.txDeg;
                nTx++;
                if (!Double.isNaN(t.rangeIn)) {
                    sumRange += t.rangeIn;
                    nRange++;
                }
            }

            // Average this frame's group mean with the last few frames, so a frame
            // that saw one tag instead of two does not yank the aim across.
            Double bearing;
            if (nTx > 0) {
                bearingWin[bearingNext] = sumTx / nTx;
                bearingNext = (bearingNext + 1) % AIM_SMOOTH_FRAMES;
                if (bearingCount < AIM_SMOOTH_FRAMES) bearingCount++;
                lastTagFrameMs = now;
                double s = 0;
                for (int i = 0; i < bearingCount; i++) s += bearingWin[i];
                bearing = s / bearingCount;
            } else {
                // No tag this frame: stop aiming (null) rather than steer on stale
                // data. Forget the window if the gap drags on, so an old angle cannot
                // bias what we re-acquire.
                bearing = null;
                if (now - lastTagFrameMs > AIM_SMOOTH_RESET_MS) {
                    bearingCount = 0; bearingNext = 0;
                    rangeCount = 0; rangeNext = 0;
                }
            }

            Double tagRange;
            if (nRange > 0) {
                rangeWin[rangeNext] = sumRange / nRange;
                rangeNext = (rangeNext + 1) % AIM_SMOOTH_FRAMES;
                if (rangeCount < AIM_SMOOTH_FRAMES) rangeCount++;
                double s = 0;
                for (int i = 0; i < rangeCount; i++) s += rangeWin[i];
                tagRange = s / rangeCount;
            } else {
                tagRange = null;
            }

            // Latch every frame a tag is seen, so the count and the ids survive even
            // if one is only visible for a moment.
            if (!tags.isEmpty()) {
                tagsSeenFrames++;
                lastTagSeen = describeGroup(tags);
            }

            // Range for the launcher: the group's, remembered for a moment so a
            // single blinking frame does not drop the wheel's speed to zero.
            if (tagRange != null) {
                lastRangeIn = tagRange;
                lastRangeMs = now;
            }
            Double rangeIn = (lastRangeIn != null && now - lastRangeMs <= RANGE_HOLD_MS)
                    ? lastRangeIn : null;

            AutoAim.Aim command = aim.update(bearing, now);
            if (command.hasTarget && aimStart == 0L) aimStart = now;

            // Before the first tag: search. After it: hand the turn to AutoAim, so a
            // lost tag just stops the turn instead of hunting.
            boolean searching = !command.hasTarget && aimStart == 0L;
            double turn = searching ? SEARCH_SIGN * SEARCH_TURN_POWER : command.turn;

            // Aim/search turn only -- forward is always zero here.
            drive.driveRobotCentric(0.0, turn);

            shooter.update(rangeIn);

            // Fire when CLOSE, not perfect. The tag group mean jumps 0..9 deg as
            // members blink, so AutoAim's strict settled (centered 250 ms, or 6
            // reversals on sign flips) rarely happens -- same-side jumping never
            // flips sign so the reversal counter stays at 0. Gate on the wider
            // window instead.
            // LATCHED, and the spin-up test is left to ShooterControls, which owns the
            // feed gate: it starts a feed only on a wheel measured at speed and then
            // holds that feed open. Testing isSpunUp() HERE as well made the two gates
            // fight -- the wheel dips under a ball's load, this one withdraws the
            // request, and the windmill stutters. Ask once, keep asking, and let one
            // place decide.
            boolean aimed = command.hasTarget
                    && Math.abs(command.bearingDeg) <= FIRE_TOLERANCE_DEG;
            boolean wantFeed = aimed || feedStart != 0L;
            shooter.requestFeed(wantFeed, false);
            if (aimed) everAimed = true;

            // Latch off what the feed actually DID, not what was asked for, so the timer
            // starts with the first ball rather than with the first request.
            if (shooter.getFeedPower() > 0 && feedStart == 0L) feedStart = now;

            // --- telemetry: what the aim loop and the wheel are doing ---
            telemetry.addData("tag", tags.isEmpty() ? "none in view"
                    : tags.size() + " in view " + describeGroup(tags));
            telemetry.addData("bearing", command.hasTarget
                    ? String.format(Locale.US, "%+.1f deg", command.bearingDeg) : "--");
            telemetry.addData("aim", searching ? "SEARCHING"
                    : !command.hasTarget ? "no target"
                    : aimed ? "CLOSE -> firing"
                    : "turning");
            telemetry.addData("tags seen", "%d frame(s)%s", tagsSeenFrames,
                    lastTagSeen == null ? "" : "   last: " + lastTagSeen);
            telemetry.addData("range", rangeIn == null
                    ? "no reading" : String.format(Locale.US, "%.1f in", rangeIn));
            telemetry.addData("wheel", "%s  asked %.0f  actual %.0f ticks/s  %s",
                    shooter.velocitySource(), shooter.getCommandedVelocity(),
                    shooter.getVelocity(), shooter.isSpunUp() ? "READY" : "spinning up");
            telemetry.addData("feed", shooter.feedState());
            telemetry.addLine();
            telemetry.addLine("If it turns AWAY from the tag, flip AutoAim.TURN_SIGN.");
            telemetry.update();

            // Done: fed for the full window.
            if (feedStart != 0L && now - feedStart >= FEED_DURATION_MS) {
                drive.stop();
                return String.format(Locale.US, "fired -- fed %.0f s after aiming",
                        FEED_DURATION_MS / 1000.0);
            }

            // Searched all the way round and still nothing: stop rather than spin.
            if (searching) {
                double swept = (Math.abs(drive.leftTicks() - searchLeftStart)
                        + Math.abs(drive.rightTicks() - searchRightStart)) / 2.0;
                if (swept >= searchTicks) {
                    drive.stop();
                    return "STOPPED: searched a full turn, no tag -- check camera aim / pipeline";
                }
            }

            // Had a tag but could not use it. Each exit says which half failed.
            if (feedStart == 0L && aimStart != 0L && now - aimStart >= AIM_TIMEOUT_MS) {
                drive.stop();
                if (!everAimed) {
                    return "STOPPED: never got within FIRE_TOLERANCE_DEG -- check AutoAim.TURN_SIGN / KP";
                }
                return "STOPPED: close but wheel never reached speed -- check launcher / battery";
            }

            idle();
        }

        drive.stop();
        return "stopped";
    }

    /**
     * Drive both sides straight ahead by the same number of ticks and wait until
     * they get there, or until the timeout or an encoder failure. Returns a status
     * string for the Driver Station.
     *
     * The same per-side encoder check as {@link ForwardAndTurnAuto#runMove}: a side
     * that was asked to move but has counted nothing after the grace period has a
     * cable problem, and no amount of driving will ever satisfy it. Straight-line
     * only, so both sides get the same delta.
     */
    private String driveForward(String label, int distanceTicks) {
        int delta = (int) (ForwardAndTurnAuto.FORWARD_SIGN * distanceTicks);
        int leftStart = drive.leftTicks();
        int rightStart = drive.rightTicks();

        drive.driveToTicks(leftStart + delta, rightStart + delta, DRIVE_POWER);

        long begin = System.currentTimeMillis();
        while (opModeIsActive()) {
            int leftMoved = drive.leftTicks() - leftStart;
            int rightMoved = drive.rightTicks() - rightStart;
            long elapsed = System.currentTimeMillis() - begin;

            boolean leftDead = leftMoved == 0;
            boolean rightDead = rightMoved == 0;
            boolean encodersDead = elapsed > ENCODER_STARTUP_GRACE_MS
                    && (leftDead || rightDead);

            telemetry.addData("move", label);
            telemetry.addData("  left", "%d / %d ticks%s", leftMoved, delta,
                    leftDead ? "   <-- no encoder reading" : "");
            telemetry.addData("  right", "%d / %d ticks%s", rightMoved, delta,
                    rightDead ? "   <-- no encoder reading" : "");
            telemetry.addData("  elapsed", "%d ms", elapsed);
            if (encodersDead) {
                telemetry.addLine("drive encoders not reading -- check the cables");
            }
            telemetry.update();

            if (encodersDead) {
                drive.stop();
                settle();
                return "STOPPED: " + (leftDead && rightDead ? "both"
                        : leftDead ? "left" : "right")
                        + " drive encoder not reading -- check the cables";
            }

            boolean arrived = Math.abs(delta - leftMoved) <= TOLERANCE_TICKS
                    && Math.abs(delta - rightMoved) <= TOLERANCE_TICKS;
            if (arrived) {
                drive.stop();
                settle();
                return REACHED;
            }
            if (elapsed > FORWARD_TIMEOUT_MS) {
                drive.stop();
                settle();
                return "TIMED OUT -- check geometry";
            }
            idle();
        }

        drive.stop();
        return "stopped";
    }

    /** Let the wheels actually come to rest before we read position or start aiming. */
    private void settle() {
        sleep(150);
    }

    /** Trim a trailing ".0" so "1.0" reads as "1" and "1.5" keeps its fraction. */
    private static String format(double value) {
        return (value == Math.floor(value))
                ? String.valueOf((long) value)
                : String.valueOf(value);
    }

    /** The tag IDs in view, e.g. "[32, 33]", for the latched "last seen" line. */
    private static String describeGroup(List<LimelightHiveTracker.Tag> tags) {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < tags.size(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(tags.get(i).id);
        }
        return sb.append("]").toString();
    }
}
