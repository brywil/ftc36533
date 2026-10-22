package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Shoot from where the robot already is, then drive: forward 1 square, left 90,
 * forward 1 square, right 90, forward 2.5 squares, left 90, forward 1 square, then
 * back up 0.2 squares.
 *
 * This is the BLUE path. {@link ShootThenDriveRedAuto} is the SAME routine, same turns,
 * started from this start turned 180 degrees about the field centre: the BIOBUZZ loading
 * zones are placed by rotation, not mirrored (manual Figures 9-3, 9-5; see the red file).
 * If you retune one, make the identical change in the other.
 *
 * NO AIMING. This OpMode assumes the robot is already lined up on the goal when START
 * is pressed. It does not use the Limelight at all -- no tag search, no bearing, no
 * steering -- so it cannot be thrown off by which tags happen to be in view. The
 * launcher just spins up to a fixed speed and feeds.
 *
 * SEQUENCE, on START:
 *   1. ask the launcher for LAUNCH_VELOCITY immediately;
 *   2. wait for the encoder to REPORT the wheel actually at speed, capped so a broken
 *      wheel cannot hang the OpMode;
 *   3. feed for FEED_DURATION_MS (5 s), with the intake running so a ball touching it
 *      still gets pulled in;
 *   4. drive at 50% power: forward 1, left 90, forward 1, right 90, forward 2.5,
 *      left 90, forward 1, then reverse 0.2;
 *   5. stop and report.
 *
 * The drive is plain dead reckoning with {@link ForwardAndTurnAuto}'s measured
 * geometry, so the tile and turn calibration lives in one place. Forward uses
 * ForwardAndTurnAuto.FORWARD_SIGN, the direction that was found to be "forward" on
 * this robot in OpMode 8.
 *
 * WHAT LEFT/RIGHT MEAN HERE. The drivebase's +turn is clockwise, so a right turn is
 * positive and a left turn is negative: see TURN_RIGHT_SIGN / TURN_LEFT_SIGN.
 *
 * SAFETY: wheels off the ground for the first run. The launcher spins on START -- keep
 * clear of the muzzle. Every move has a timeout and stops if a drive encoder is dead.
 */
@Autonomous(name = "11. Auto: Shoot, Then Drive (Blue)", group = "Auto")
public class ShootThenDriveAuto extends LinearOpMode {

    // --- Shooting -----------------------------------------------------------

    /** Launcher wheel speed for the shot, ticks/s. Same as the other autos. */
    public static final double LAUNCH_VELOCITY = 1300.0;

    /**
     * Cap on waiting for the wheel to report at-speed before feeding anyway. Raised
     * from 3 s because the wheel was being fed before it finished spinning up (shots
     * came out short). If the telemetry still says "launcher at speed: NO" at 5 s, the
     * wheel genuinely cannot reach LAUNCH_VELOCITY -- lower that value, charge the
     * battery, or check the launcher's encoder cable.
     */
    public static final long SPIN_UP_MAX_WAIT_MS = 5000;

    /** How long to run the windmill, pushing balls into the wheel. */
    public static final long FEED_DURATION_MS = 5000;

    /**
     * Run the intake while feeding, at this power. The windmill pushes balls in from
     * the hopper; running the intake at the same time keeps pulling from the field and
     * can dislodge a ball sitting on the intake. goBILDA's own launch routine adds
     * intake power for exactly this reason.
     */
    public static final double FEED_INTAKE_POWER = 1.0;

    // --- Post-shot movement -------------------------------------------------

    /** The four forward legs, in squares, in drive order. */
    public static final double FORWARD_1_SQUARES = 1.0;
    public static final double FORWARD_2_SQUARES = 1.0;
    public static final double FORWARD_3_SQUARES = 2.5;
    public static final double FORWARD_4_SQUARES = 1.0;

    /** Reverse this many squares at the very end. */
    public static final double BACKWARD_FINAL_SQUARES = 0.2;

    /** How far each turn goes, in degrees. */
    public static final double TURN_DEG = 90.0;

    /** +1 is clockwise, -1 counter-clockwise. Turn order is left, right, left. */
    public static final double TURN_RIGHT_SIGN = 1.0;
    public static final double TURN_LEFT_SIGN = -1.0;

    /** Movement power, 50% as requested, for both driving and turning. */
    public static final double MOVEMENT_POWER = 0.5;

    public static final int TOLERANCE_TICKS = 15;
    public static final long FORWARD_TIMEOUT_MS = 8000;
    public static final long TURN_TIMEOUT_MS = 5000;
    public static final long ENCODER_STARTUP_GRACE_MS = 500;

    private static final String REACHED = "reached";

    private TankDrivebase drive;
    private Intake intake;
    private ShooterControls shooter;
    private double ticksPerInch;

    @Override
    public void runOpMode() {
        drive = new TankDrivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new ShooterControls(new Shooter(hardwareMap));

        ticksPerInch = ForwardAndTurnAuto.TICKS_PER_WHEEL_REV
                / (Math.PI * ForwardAndTurnAuto.WHEEL_DIAMETER_IN);

        telemetry.addLine("Auto: shoot (no aim), then fwd 1, left 90, fwd 1, right 90, fwd 2.5, left 90, fwd 1.");
        telemetry.addLine("Robot must ALREADY be lined up. WHEELS OFF THE GROUND first run.");
        telemetry.addLine();
        telemetry.addData("launcher", shooter.shooter().hasLauncher() ? "OK" : "MISSING");
        telemetry.addData("windmill", shooter.shooter().hasWindmill() ? "OK" : "MISSING");
        telemetry.addData("movement power", "%.0f%%", MOVEMENT_POWER * 100);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // --- 1-3: spin up, wait for at-speed, feed ---
        shooter.setManual(true);
        shooter.setArmed(true);
        shooter.update(null);            // manual: command the manual speed now

        long spinBegin = System.currentTimeMillis();
        boolean ready = false;
        while (opModeIsActive()) {
            ready = shooter.isSpunUp();
            long waited = System.currentTimeMillis() - spinBegin;
            telemetry.addData("spinning up launcher", "%.0f / %.0f ticks/s  (%d ms)",
                    shooter.getVelocity(), shooter.getCommandedVelocity(), waited);
            telemetry.update();
            if (ready || waited >= SPIN_UP_MAX_WAIT_MS) break;
            idle();
        }

        telemetry.addData("launcher at speed", ready
                ? "yes" : "NO -- timed out, check battery / encoder cable");
        telemetry.update();

        // Feed for FEED_DURATION_MS, with the intake running too. The intake pulls
        // from the field and can free a ball resting against it, while the windmill
        // pushes into the wheel.
        shooter.shooter().setFeed(1.0);
        intake.setPower(FEED_INTAKE_POWER);
        long feedBegin = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - feedBegin < FEED_DURATION_MS) {
            telemetry.addData("feeding", "%d / %d ms  (intake on)",
                    (int) (System.currentTimeMillis() - feedBegin), FEED_DURATION_MS);
            telemetry.update();
            idle();
        }
        shooter.shooter().setFeed(0.0);
        intake.stop();
        shooter.stop();
        sleep(300);                       // let the wheel coast off before driving

        // --- 4: post-shot movement at 50% ---
        String fwd1 = "", turnL = "", fwd2 = "", turnR1 = "", fwd3 = "", turnL2 = "", fwd4 = "", lastBack = "";
        if (opModeIsActive()) {
            fwd1 = driveForward("after: forward 1", FORWARD_1_SQUARES);
            turnL = turn("after: turn left 90", TURN_LEFT_SIGN);
            fwd2 = driveForward("after: forward 1", FORWARD_2_SQUARES);
            turnR1 = turn("after: turn right 90", TURN_RIGHT_SIGN);
            fwd3 = driveForward("after: forward 2.5", FORWARD_3_SQUARES);
            turnL2 = turn("after: turn left 90", TURN_LEFT_SIGN);
            fwd4 = driveForward("after: forward 1", FORWARD_4_SQUARES);
            lastBack = driveBackward("after: backward 0.2", BACKWARD_FINAL_SQUARES);
        }

        drive.stop();
        shooter.stop();

        telemetry.addData("launch", ready ? "fired" : "wheel never reached speed");
        telemetry.addData("forward 1", fwd1);
        telemetry.addData("turn left 90", turnL);
        telemetry.addData("forward 1", fwd2);
        telemetry.addData("turn right 90", turnR1);
        telemetry.addData("forward 2.5", fwd3);
        telemetry.addData("turn left 90", turnL2);
        telemetry.addData("forward 1", fwd4);
        telemetry.addData("backward 0.2", lastBack);
        telemetry.update();
        sleep(2000);
    }

    private int turnTicks() {
        return (int) Math.round(
                (TURN_DEG / 360.0) * Math.PI
                * ForwardAndTurnAuto.TRACK_WIDTH_IN * ticksPerInch);
    }

    /** Ticks to drive physically forward by this many squares (FORWARD_SIGN applied). */
    private int forwardTicks(double squares) {
        return (int) Math.round(ForwardAndTurnAuto.FORWARD_SIGN * squares
                * ForwardAndTurnAuto.TILE_IN * ticksPerInch);
    }

    /** Drive physically forward this many squares at MOVEMENT_POWER. */
    private String driveForward(String label, double squares) {
        int ticks = forwardTicks(squares);
        return runMove(label + " (" + format(squares) + " sq)", ticks, ticks,
                MOVEMENT_POWER, FORWARD_TIMEOUT_MS);
    }

    /** Drive physically backward this many squares: the negative of forward. */
    private String driveBackward(String label, double squares) {
        int ticks = -forwardTicks(squares);
        return runMove(label + " (" + format(squares) + " sq)", ticks, ticks,
                MOVEMENT_POWER, FORWARD_TIMEOUT_MS);
    }

    /** Spin in place by TURN_DEG in the given direction (+1 clockwise, -1 CCW). */
    private String turn(String label, double sign) {
        int ticks = (int) (sign * turnTicks());
        return runMove(label, ticks, -ticks, MOVEMENT_POWER, TURN_TIMEOUT_MS);
    }

    /** Same per-side encoder-checked move as the other autos. */
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

    /** Trim a trailing ".0" so "3.0" reads as "3". */
    private static String format(double value) {
        return (value == Math.floor(value))
                ? String.valueOf((long) value)
                : String.valueOf(value);
    }
}
