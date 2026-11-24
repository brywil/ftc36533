package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;
import java.util.Locale;

/**
 * Spin the launcher the whole time, and when OUR HIVE CELL reaches HIGH, run the intake and
 * the indexer (windmill) for a few seconds, then drive.
 *
 * THIS FILE IS NOT AN OPMODE. {@link HiveHighShootDriveAutoRed} and
 * {@link HiveHighShootDriveAutoBlue} are the OpModes; the only thing they decide is
 * which cluster of tags is the CELL this robot scores into.
 *
 * RED AND BLUE WATCH DIFFERENT-NAMED CELLS, AND THAT IS CORRECT. The field is
 * point-symmetric, not mirrored, so the HIVE is staged the opposite way round for each
 * alliance: RED starts with its AUDIENCE-side CELL up, BLUE starts with its REAR/scoring
 * side up (manual Fig 10-2). The partner tips it after START, so:
 *
 *   RED  support robot (on the rear side)  waits for RED SCORING (30-33) to come UP;
 *   BLUE support robot (on the audience side) waits for BLUE AUDIENCE (38-41) to come UP.
 *
 * Watching the same side for both alliances would make one of them fire at the staged
 * pose, before any tip, at the wrong CELL.
 *
 * WHY WAIT AT ALL. The HIVE is a seesaw that only rests at two endpoints. A TIP is the
 * down-CELL becoming the up-CELL and settling there -- the Limelight reports that as a
 * settled observation in {@link HiveGeometry.Slot.HIGH}. Feeding before it gets there
 * throws at a moving target. This waits for a settled HIGH, using the very same gate as
 * {@link LimelightHiveTracker} / {@link HiveGate} (tilt AND height agreeing).
 *
 * SETTLED HIGH IS THE TRIGGER; LOW FIRST IS ONLY EXTRA EVIDENCE. It used to insist on
 * seeing the CELL settled LOW before the tip, and that is what stopped both support
 * autos from firing: at START the camera is looking at the goal from an angle, the
 * watched CELL is behind the field structure, or the robot is still being placed -- so
 * LOW is never seen. By the time the tags do appear, the CELL is already up, and the
 * auto sat there through the whole match and skipped the feed.
 *
 * BOTH ALLIANCES WATCH A CELL THAT STARTS **DOWN**, which is what makes firing on the
 * first settled HIGH safe: RED watches RED SCORING (down at staging) and BLUE watches
 * BLUE AUDIENCE (down at staging) -- see the subclasses and manual Fig 10-2. So a HIGH
 * on first sight means either the partner tipped the HIVE while we were not looking, or
 * it was already tipped before START; either way our scoring CELL is UP and it is the
 * right one to shoot into. The LOW-first path is still reported when it is seen, because
 * that is the proof the tip really happened.
 *
 * IF IT SEES NOTHING AT ALL. It fires the feed anyway, two ways:
 *
 *   - nothing of our CELL in view for FIRE_BLIND_AFTER_MS (8 s) -- a camera problem, a
 *     CELL empty from the last match, or a partner who tipped at once. The feed is the
 *     only thing that scores, and a skipped feed scores nothing, so the robot shoots
 *     what it has and still runs the drive;
 *   - our CELL in view but never settling HIGH by WAIT_FOR_HIGH_MS (12 s) -- a CELL held
 *     mid-tip, or a gate that refuses an oblique view.
 *
 * THE TRADE-OFF, STATED PLAINLY. Firing blind can throw balls at a CELL that is still
 * down, which wastes them. That is accepted deliberately: the alternative (the 2026-11-20
 * behaviour) was to sit out the whole AUTO and score nothing whenever the camera did not
 * see the start pose, which is strictly worse. The report line says which path was taken
 * ("tip seen", "HIGH seen first", "no tags -- fired anyway"), so a bad run can be told
 * apart from a lucky one. The 8 s / 12 s caps also keep feed + drive inside the 30 s.
 *
 * SEQUENCE, on START:
 *   1. spin the launcher up and KEEP it spinning for the whole OpMode;
 *   2. wait for a settled HIGH -- or, failing that, up to 8 s with nothing in view, or
 *      12 s with the CELL in view but never settling;
 *   3. run the intake AND the indexer (windmill) for FEED_DURATION_MS -- always;
 *   4. drive at MOVEMENT_POWER: forward 0.5, RIGHT 90, forward 2, then reverse 0.2;
 *   5. stop and report.
 *
 * WHAT RIGHT MEANS. The drivebase's +turn is clockwise, so a right turn is
 * TURN_RIGHT_SIGN = +1. The request was the same movement for both alliances, so there
 * is no mirror here (unlike {@link ShootThenDriveRedAuto}).
 *
 * SAFETY: the launcher spins from START -- keep clear of the muzzle. Wheels off the
 * ground for the first run. Every move has a timeout and stops on a dead drive encoder.
 */
public abstract class HiveHighShootDriveAutoBase extends LinearOpMode {

    /** Which cluster is THIS robot's scoring CELL. Answered by the subclasses. */
    protected abstract HiveGeometry.Cell scoringCell();

    // --- Wait -----------------------------------------------------------------

    /**
     * How long to keep waiting for a settled HIGH while our CELL is actually in view.
     * When it runs out we fire anyway: a skipped feed scores nothing, and the 5 s feed
     * plus the drive still have to fit in the 30 s AUTO.
     */
    public static final long WAIT_FOR_HIGH_MS = 12000;

    /**
     * If NONE of our CELL has been in view for this long, the tip will never be seen.
     * Fire blind instead of waiting out WAIT_FOR_HIGH_MS at a camera that sees nothing.
     * 8 s leaves room for a partner who tips late, and for the feed + drive after it.
     */
    public static final long FIRE_BLIND_AFTER_MS = 8000;

    // --- Shooting -------------------------------------------------------------

    public static final double LAUNCH_VELOCITY = 1200.0;
    public static final long FEED_DURATION_MS = 5000;
    public static final double FEED_INTAKE_POWER = 1.0;

    // --- Movement -------------------------------------------------------------

    public static final double FORWARD_FIRST_SQUARES = 0.5;
    public static final double FORWARD_SECOND_SQUARES = 2.0;
    public static final double BACKWARD_FINAL_SQUARES = 0.2;
    public static final double TURN_RIGHT_DEG = 90.0;
    public static final double TURN_RIGHT_SIGN = 1.0;   // clockwise

    public static final double MOVEMENT_POWER = 0.5;
    public static final int TOLERANCE_TICKS = 15;
    public static final long FORWARD_TIMEOUT_MS = 8000;
    public static final long TURN_TIMEOUT_MS = 5000;
    public static final long ENCODER_STARTUP_GRACE_MS = 500;

    private static final String REACHED = "reached";

    private TankDrivebase drive;
    private Intake intake;
    private ShooterControls shooter;
    private LimelightHiveTracker tracker;
    private double ticksPerInch;

    @Override
    public void runOpMode() {
        drive = new TankDrivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new ShooterControls(new Shooter(hardwareMap));
        tracker = new LimelightHiveTracker(hardwareMap, "limelight", 1);

        ticksPerInch = ForwardAndTurnAuto.TICKS_PER_WHEEL_REV
                / (Math.PI * ForwardAndTurnAuto.WHEEL_DIAMETER_IN);

        telemetry.addLine("Auto: spin launcher, wait for " + scoringCell().clusterName + " to be UP, feed, then drive.");
        telemetry.addLine("WHEELS OFF THE GROUND for the first run. Launcher spins on START.");
        telemetry.addLine();
        telemetry.addData("limelight", tracker.hasLimelight() ? "OK" : "MISSING");
        telemetry.addData("watch cell", scoringCell().clusterName);
        telemetry.addData("launcher", shooter.shooter().hasLauncher() ? "OK" : "MISSING");
        telemetry.addData("windmill", shooter.shooter().hasWindmill() ? "OK" : "MISSING");
        telemetry.addData("gate", shooter.shooter().hasGate() ? "OK" : "MISSING");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        tracker.start();

        // Spin the launcher up and leave it spinning for the whole OpMode. It is
        // velocity-controlled, so this one call holds the speed; nothing stops it until
        // the very end, so it is already at speed whenever the HIVE reaches HIGH.
        shooter.setManual(true);
        shooter.setArmed(true);
        // Pin this auto's own speed; otherwise it inherits the teleop's default.
        shooter.setManualVelocity(LAUNCH_VELOCITY);
        shooter.update(null);

        Tip tip = waitForTip();
        boolean fed = false;

        // SHOOT UNLESS THE OPMODE WAS STOPPED. A settled HIGH, a HIGH with no LOW before
        // it, or either give-up timer all end in the feed: skipping it scores nothing,
        // and the feed plus the drive still have to fit inside the 30 s AUTO.
        if (tip != Tip.STOPPED) {
            // The wheel has been running the whole time, so there is no spin-up wait --
            // run the intake AND the indexer (windmill) together for FEED_DURATION_MS.
            shooter.shooter().setFeed(1.0);
            intake.setPower(FEED_INTAKE_POWER);
            long feedBegin = System.currentTimeMillis();
            while (opModeIsActive()
                    && System.currentTimeMillis() - feedBegin < FEED_DURATION_MS) {
                telemetry.addData("feeding (intake + indexer)", "%d / %d ms",
                        (int) (System.currentTimeMillis() - feedBegin), FEED_DURATION_MS);
                telemetry.addData("  wheel", "%.0f / %.0f ticks/s",
                        shooter.getVelocity(), shooter.getCommandedVelocity());
                telemetry.update();
                idle();
            }
            shooter.shooter().setFeed(0.0);
            intake.stop();
            fed = true;
        }

        // The launcher keeps spinning through the movement, per "spinning up the entire
        // time"; it is stopped at the very end below.
        sleep(300);

        // --- movement (same for both alliances) ---
        String fwd1 = "", turnR = "", fwd2 = "", back = "";
        if (opModeIsActive()) {
            fwd1 = driveForward("forward 0.5", FORWARD_FIRST_SQUARES);
            turnR = turn("turn right 90", TURN_RIGHT_SIGN);
            fwd2 = driveForward("forward 2", FORWARD_SECOND_SQUARES);
            back = driveBackward("back 0.2", BACKWARD_FINAL_SQUARES);
        }

        drive.stop();
        intake.stop();
        shooter.stop();
        tracker.close();

        telemetry.addData("tip", tip.message());
        telemetry.addData("feed", fed ? "fed 5 s" : "not fed");
        telemetry.addData("forward 0.5", fwd1);
        telemetry.addData("turn right 90", turnR);
        telemetry.addData("forward 2", fwd2);
        telemetry.addData("back 0.2", back);
        telemetry.update();
        sleep(2000);
    }

    /**
     * How the wait for the tip ended. Every value except STOPPED means "fire": the point
     * of this auto is to score, and a skipped feed scores nothing.
     */
    private enum Tip {
        /** Our CELL settled LOW, then settled HIGH: a real TIP, and it was watched. */
        TIPPED,
        /** First sight of our CELL was already settled HIGH -- tipped unseen. Both
         *  support autos watch a CELL that starts DOWN, so this is still our up CELL. */
        HIGH_ONLY,
        /** Nothing of our CELL in view for FIRE_BLIND_AFTER_MS: fire blind. */
        BLIND,
        /** Our CELL was in view but never settled HIGH by WAIT_FOR_HIGH_MS: fire anyway. */
        NEVER_HIGH,
        /** The OpMode was stopped while waiting; do not start the feed. */
        STOPPED;

        /** One short line for the run report. */
        String message() {
            switch (this) {
                case TIPPED:     return "tip seen (LOW -> HIGH)";
                case HIGH_ONLY:  return "HIGH seen first -- fired (cell starts DOWN)";
                case BLIND:      return "no tags -- fired anyway (blind)";
                case NEVER_HIGH: return "never settled HIGH -- fired anyway";
                default:         return "stopped before the feed";
            }
        }
    }

    /**
     * Wait until our target CELL is settled HIGH -- the TIP -- or until one of the two
     * give-up deadlines passes, and report which happened.
     *
     * A SETTLED HIGH IS ENOUGH, WITH OR WITHOUT A LOW BEFORE IT. Requiring the LOW was
     * what stopped this auto from firing: at START the camera often cannot see the
     * watched CELL at all, so by the time the tags appear the CELL is already up. Since
     * BOTH support autos watch a CELL that starts DOWN, a HIGH on first sight can only
     * mean our scoring CELL is up -- shoot it.
     *
     * The two deadlines, and why they are different:
     *
     *   - nothing of our CELL in view for FIRE_BLIND_AFTER_MS (8 s): the tip was never
     *     going to be seen, so stop waiting and shoot;
     *   - our CELL in view but never settling HIGH by WAIT_FOR_HIGH_MS (12 s): a CELL
     *     held mid-tip or a gate refusing an oblique view, so stop waiting and shoot.
     *
     * A CELL that blinks out and back does not restart the blind timer -- only time with
     * nothing in view counts -- so a flaky tag does not push the start of the feed late.
     */
    private Tip waitForTip() {
        long begin = System.currentTimeMillis();
        long lastSeenAt = begin;
        boolean sawLow = false;
        boolean sawAny = false;

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();
            long elapsed = now - begin;

            List<LimelightHiveTracker.Observation> obs = tracker.observe();
            boolean inView = false;
            boolean high = false;
            String lastSeen = "none in view";
            for (LimelightHiveTracker.Observation o : obs) {
                if (o.cell != scoringCell()) continue;
                inView = true;
                lastSeen = String.format(Locale.US, "tilt %+.1f  slot %s  %s%s",
                        o.tiltDeg, o.slot,
                        o.settled ? "settled" : "moving",
                        o.rejectReason == null ? "" : "  (" + o.rejectReason + ")");
                if (o.settled && o.slot == HiveGeometry.Slot.LOW) sawLow = true;
                if (o.settled && o.slot == HiveGeometry.Slot.HIGH) high = true;
            }
            if (inView) {
                sawAny = true;
                lastSeenAt = now;
            }

            // Settled HIGH: our scoring CELL is up. LOW first is reported, not required.
            if (high) return sawLow ? Tip.TIPPED : Tip.HIGH_ONLY;

            long unseen = now - lastSeenAt;
            long shown = Math.min(elapsed, WAIT_FOR_HIGH_MS);

            telemetry.addData("waiting for TIP", "%d / %d ms  (LOW seen: %s)",
                    shown, WAIT_FOR_HIGH_MS, sawLow ? "yes" : "not yet");
            telemetry.addData("  " + scoringCell().clusterName, lastSeen);
            telemetry.addData("  fire blind in", sawAny ? "-- (tags in view)" : "%d ms",
                    Math.max(0, FIRE_BLIND_AFTER_MS - unseen));
            telemetry.update();

            if (!sawAny && unseen >= FIRE_BLIND_AFTER_MS) return Tip.BLIND;
            if (elapsed >= WAIT_FOR_HIGH_MS) return Tip.NEVER_HIGH;
            idle();
        }
        return Tip.STOPPED;
    }

    private int turnTicks() {
        return (int) Math.round(
                (TURN_RIGHT_DEG / 360.0) * Math.PI
                * ForwardAndTurnAuto.TRACK_WIDTH_IN * ticksPerInch);
    }

    private int forwardTicks(double squares) {
        return (int) Math.round(ForwardAndTurnAuto.FORWARD_SIGN * squares
                * ForwardAndTurnAuto.TILE_IN * ticksPerInch);
    }

    private String driveForward(String label, double squares) {
        int ticks = forwardTicks(squares);
        return runMove(label, ticks, ticks, MOVEMENT_POWER, FORWARD_TIMEOUT_MS);
    }

    private String driveBackward(String label, double squares) {
        int ticks = -forwardTicks(squares);
        return runMove(label, ticks, ticks, MOVEMENT_POWER, FORWARD_TIMEOUT_MS);
    }

    private String turn(String label, double sign) {
        int ticks = (int) (sign * turnTicks());
        return runMove(label, ticks, -ticks, MOVEMENT_POWER, TURN_TIMEOUT_MS);
    }

    private String runMove(String label, int leftDelta, int rightDelta,
                           double power, long timeoutMs) {
        int leftStart = drive.leftTicks();
        int rightStart = drive.rightTicks();

        drive.driveToTicks(leftStart + leftDelta, rightStart + rightDelta, power);

        long begin = System.currentTimeMillis();
        while (opModeIsActive()) {
            int leftMoved = drive.leftTicks() - leftStart;
            int rightMoved = drive.rightTicks() - rightStart;
            long elapsed = System.currentTimeMillis() - begin;

            boolean leftDead = leftDelta != 0 && leftMoved == 0;
            boolean rightDead = rightDelta != 0 && rightMoved == 0;
            boolean encodersDead = elapsed > ENCODER_STARTUP_GRACE_MS
                    && (leftDead || rightDead);

            telemetry.addData("move", label);
            telemetry.addData("  left", "%d / %d ticks", leftMoved, leftDelta);
            telemetry.addData("  right", "%d / %d ticks", rightMoved, rightDelta);
            telemetry.update();

            if (encodersDead) {
                drive.stop();
                sleep(150);
                return "STOPPED: drive encoder not reading -- check the cables";
            }
            boolean arrived = Math.abs(leftDelta - leftMoved) <= TOLERANCE_TICKS
                    && Math.abs(rightDelta - rightMoved) <= TOLERANCE_TICKS;
            if (arrived) {
                drive.stop();
                sleep(150);
                return REACHED;
            }
            if (elapsed > timeoutMs) {
                drive.stop();
                sleep(150);
                return "TIMED OUT -- check geometry";
            }
            idle();
        }

        drive.stop();
        return "stopped";
    }
}
