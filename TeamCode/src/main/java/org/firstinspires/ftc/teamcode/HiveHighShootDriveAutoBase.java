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
 * throws at a moving target. This waits for that exact transition, using the very same
 * gate as {@link LimelightHiveTracker} / {@link HiveGate} (tilt AND height agreeing), and
 * requires the CELL to have been seen settled LOW first, so a strange initial state
 * cannot be mistaken for a tip.
 *
 * IF IT NEVER HAPPENS. It waits up to WAIT_FOR_HIGH_MS (20 s); when that runs out it
 * SKIPS the feed and does the movement anyway, so the robot still leaves. A HIVE that
 * never reports HIGH is more likely a camera/tag problem than a scoring opportunity, and
 * firing blind just wastes balls. Telemetry says which path it took.
 *
 * SEQUENCE, on START:
 *   1. spin the launcher up and KEEP it spinning for the whole OpMode;
 *   2. wait for our CELL to go SETTLED LOW -> SETTLED HIGH (a TIP), up to 20 s;
 *   3. when it reaches HIGH: run the intake AND the indexer (windmill) for FEED_DURATION_MS;
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

    /** How long to wait for our CELL to reach HIGH before giving up. */
    public static final long WAIT_FOR_HIGH_MS = 20000;

    // --- Shooting -------------------------------------------------------------

    public static final double LAUNCH_VELOCITY = 1300.0;
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

        telemetry.addLine("Auto: spin launcher, wait for " + scoringCell() + " to reach HIGH, feed, then drive.");
        telemetry.addLine("WHEELS OFF THE GROUND for the first run. Launcher spins on START.");
        telemetry.addLine();
        telemetry.addData("limelight", tracker.hasLimelight() ? "OK" : "MISSING");
        telemetry.addData("watch cell", scoringCell().clusterName);
        telemetry.addData("launcher", shooter.shooter().hasLauncher() ? "OK" : "MISSING");
        telemetry.addData("windmill", shooter.shooter().hasWindmill() ? "OK" : "MISSING");
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

        boolean high = waitForHigh();
        boolean fed = false;

        if (high) {
            // It reached HIGH: run the intake AND the indexer (windmill) together for
            // FEED_DURATION_MS. No spin-up wait needed -- the wheel has been running
            // the whole time.
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
        } else {
            telemetry.addLine("HIVE never reached HIGH in 20 s -- skipping the feed.");
            telemetry.update();
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

        telemetry.addData("TIP seen", high ? "yes" : "no -- feed skipped");
        telemetry.addData("feed", fed ? "fed 5 s" : "not fed");
        telemetry.addData("forward 0.5", fwd1);
        telemetry.addData("turn right 90", turnR);
        telemetry.addData("forward 2", fwd2);
        telemetry.addData("back 0.2", back);
        telemetry.update();
        sleep(2000);
    }

    /**
     * Wait until our target CELL has gone from settled LOW to settled HIGH -- the TIP --
     * or WAIT_FOR_HIGH_MS passes. Returns true if a full tip was seen.
     *
     * REQUIRING LOW FIRST IS THE POINT. Firing on "HIGH" alone would fire on the staged
     * pose for whichever alliance starts with that CELL up, before any tip happened. The
     * partner tips the HIVE after START and the robot scores into the CELL that comes UP,
     * so seeing it DOWN first proves the tip really occurred.
     */
    private boolean waitForHigh() {
        long begin = System.currentTimeMillis();
        boolean sawLow = false;
        while (opModeIsActive()) {
            long elapsed = System.currentTimeMillis() - begin;

            List<LimelightHiveTracker.Observation> obs = tracker.observe();
            boolean found = false;
            String lastSeen = "none in view";
            for (LimelightHiveTracker.Observation o : obs) {
                if (o.cell != scoringCell()) continue;
                lastSeen = String.format(Locale.US, "tilt %+.1f  slot %s  %s",
                        o.tiltDeg, o.slot, o.settled ? "settled" : "moving");
                if (o.settled && o.slot == HiveGeometry.Slot.LOW) {
                    sawLow = true;
                }
                if (o.settled && o.slot == HiveGeometry.Slot.HIGH && sawLow) {
                    found = true;
                    break;
                }
            }

            telemetry.addData("waiting for TIP", "%d / %d ms  (seen LOW: %s)",
                    elapsed, WAIT_FOR_HIGH_MS, sawLow ? "yes" : "not yet");
            telemetry.addData("  " + scoringCell().clusterName, lastSeen);
            telemetry.update();

            if (found) return true;
            if (elapsed >= WAIT_FOR_HIGH_MS) return false;
            idle();
        }
        return false;
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
