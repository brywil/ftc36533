package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Shoot from where the robot already is, then drive the RED-alliance path: forward 1
 * square, left 90, forward 1 square, right 90, forward 2.5 squares, left 90, forward
 * 1 square, then back up 0.2 squares -- the SAME path as the blue auto.
 *
 * WHY THE SAME TURNS, NOT MIRRORED ONES. The BIOBUZZ field is not a mirror image left
 * to right. The LOADING ZONES and GARDENS are placed with 180 degree ROTATION
 * (Competition Manual Figures 9-3 and 9-5): the red LOADING ZONE is tile A5, at the
 * REAR end of the red (left) wall, and the blue LOADING ZONE is tile F2, at the
 * AUDIENCE end of the blue (right) wall. Turning the whole field half way round carries
 * one onto the other, and a half turn does not swap left and right -- so the red copy
 * of the tested blue run is the blue run, unchanged, started from the spot you get by
 * turning blue's start half way round the field centre. (A mirrored path from the
 * mirrored start would end near A2, the wrong end of the red wall.)
 *
 * WHERE TO START IT. Take where the blue robot starts on the right wall, and its
 * heading, and turn both 180 degrees about the field centre: a blue start near the
 * audience end of the right wall becomes a red start near the REAR end of the left wall,
 * facing the opposite way. If you retune the blue auto, make the identical change here.
 *
 * NO AIMING. Assumes the robot is already lined up on the red goal at START. It does
 * not use the Limelight at all. The launcher spins up to a fixed speed and feeds.
 *
 * SEQUENCE, on START:
 *   1. ask the launcher for LAUNCH_VELOCITY immediately;
 *   2. wait for the encoder to REPORT the wheel actually at speed, capped so a broken
 *      wheel cannot hang the OpMode;
 *   3. feed for FEED_DURATION_MS (5 s), with the intake running so a ball touching it
 *      still gets pulled in;
 *   4. drive at 50% power: forward 1, left 90, forward 1, right 90, forward 2.5,
 *      left 90, forward 1, then reverse 0.2 (identical to blue);
 *   5. stop and report.
 *
 * SAFETY: wheels off the ground for the first run. The launcher spins on START -- keep
 * clear of the muzzle. Every move has a timeout and stops if a drive encoder is dead.
 */
@Autonomous(name = "12. Auto: Shoot, Then Drive (Red)", group = "Auto")
public class ShootThenDriveRedAuto extends LinearOpMode {

    // --- Shooting -----------------------------------------------------------

    /** Launcher wheel speed for the shot, ticks/s. Same as the blue auto. */
    public static final double LAUNCH_VELOCITY = 1300.0;

    /** Cap on waiting for the wheel to report at-speed before feeding anyway. */
    public static final long SPIN_UP_MAX_WAIT_MS = 5000;

    /** How long to run the windmill, pushing balls into the wheel. */
    public static final long FEED_DURATION_MS = 5000;

    /** Run the intake while feeding, so a ball touching it is still pulled in. */
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

    /** +1 is clockwise, -1 counter-clockwise. Turn order is left, right, left, as on blue. */
    public static final double TURN_RIGHT_SIGN = 1.0;
    public static final double TURN_LEFT_SIGN = -1.0;

    /** Movement power, 50%, for both driving and turning. */
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

        telemetry.addLine("RED auto: shoot, then fwd 1, left 90, fwd 1, right 90, fwd 2.5, left 90, fwd 1.");
        telemetry.addLine("Start at BLUE's start turned 180 deg about the field centre.");
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
        shooter.update(null);

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

        // Feed with the intake running.
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
        sleep(300);

        // --- 4: post-shot movement at 50%, the SAME turns as the blue auto (see the class comment) ---
        String fwd1 = "", turnL = "", fwd2 = "", turnR = "", fwd3 = "", turnL2 = "", fwd4 = "", lastBack = "";
        if (opModeIsActive()) {
            fwd1 = driveForward("after: forward 1", FORWARD_1_SQUARES);
            turnL = turn("after: turn left 90", TURN_LEFT_SIGN);
            fwd2 = driveForward("after: forward 1", FORWARD_2_SQUARES);
            turnR = turn("after: turn right 90", TURN_RIGHT_SIGN);
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
        telemetry.addData("turn right 90", turnR);
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

    private String driveForward(String label, double squares) {
        int ticks = forwardTicks(squares);
        return runMove(label + " (" + format(squares) + " sq)", ticks, ticks,
                MOVEMENT_POWER, FORWARD_TIMEOUT_MS);
    }

    private String driveBackward(String label, double squares) {
        int ticks = -forwardTicks(squares);
        return runMove(label + " (" + format(squares) + " sq)", ticks, ticks,
                MOVEMENT_POWER, FORWARD_TIMEOUT_MS);
    }

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

    private static String format(double value) {
        return (value == Math.floor(value))
                ? String.valueOf((long) value)
                : String.valueOf(value);
    }
}
