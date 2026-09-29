package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * The simplest useful Autonomous: drive two floor tiles forward, then spin 72
 * degrees clockwise. Both moves are dead-reckoned from the drive motors' encoders
 * -- there is no IMU on the kitbot, so a heading cannot be measured directly, only
 * inferred from how far each side has rolled.
 *
 * WHICH ROBOT: the goBILDA StarterBot / kitbot (two motors, left_drive and
 * right_drive). It uses {@link TankDrivebase} for both its direction constants and
 * its encoder frame, so a side that drives forward in TeleOp also drives forward
 * here. The practice bot (mecanum) should get its own version that can use the IMU.
 *
 * ---------------------------------------------------------------------------
 * THE THREE CONSTANTS THAT DECIDE WHETHER THIS LANDS WHERE YOU AIMED
 *
 * Dead reckoning multiplies errors, so these are the whole story. The defaults are
 * the stock goBILDA figures and are a starting point, not a measurement -- confirm
 * each one before trusting a score:
 *
 *   TICKS_PER_WHEEL_REV  the motor's encoder counts per OUTPUT-shaft revolution.
 *                        Read it off the goBILDA product page for your motor/gearbox
 *                        (a 5203 Yellow Jacket at 5.23:1 is 537.7). Getting this
 *                        wrong scales every distance and angle by the same factor.
 *   WHEEL_DIAMETER_IN    the driven wheel's real diameter, tape-measured. A 96 mm
 *                        goBILDA wheel is nominally 3.78 in, often quoted as "4".
 *   TRACK_WIDTH_IN       the distance between the centres of the left and right
 *                        wheels. Measure it; it is what turns wheel travel into a
 *                        rotation angle. Wrong track width is why a "72 degree"
 *                        turn comes out as 65 or 80.
 *
 * Calibrate fast: put the robot on the floor, mark the wheels, push it exactly one
 * tile, and compare the encoder ticks you read against TILE_IN * ticks-per-inch.
 * That one push catches a wrong diameter or a wrong counts-per-rev before it costs
 * you a match.
 * ---------------------------------------------------------------------------
 *
 * SAFETY: point the robot at open floor with room for two tiles plus a turn, and
 * keep clear. A dead-reckoned robot does not know it is wrong -- it drives its tick
 * count no matter what it is aimed at. If a move cannot reach its target it stops
 * on a timeout rather than running away, and says so on the Driver Station.
 */
@Autonomous(name = "8. Auto: 2 Squares Forward, Turn 72", group = "Auto")
public class ForwardAndTurnAuto extends LinearOpMode {

    // --- Field geometry -----------------------------------------------------

    /** How many floor tiles to drive forward. The request was two. */
    public static final double SQUARES = 2.0;

    /**
     * The playing field tile is 24 in. on a side (Section 9 of the manual). This is
     * one of the few numbers that is quoted rather than measured.
     */
    public static final double TILE_IN = 24.0;

    /** How far to spin, in degrees, clockwise seen from above. */
    public static final double TURN_DEG = 72.0;

    /**
     * Which way a positive turn target spins the robot. +1 is clockwise, matching
     * {@link TankDrivebase#driveRobotCentric}'s turn axis. Flip to -1 if the robot
     * spins the wrong way -- confirm it on blocks before trusting it.
     */
    public static final double TURN_SIGN = 1.0;

    // --- Robot geometry: MEASURE THESE, do not assume them ------------------

    /** Encoder counts per output-shaft revolution. 537.7 is a 5203 at 5.23:1. */
    public static final double TICKS_PER_WHEEL_REV = 537.7;

    /** Driven wheel diameter in inches. Confirm with a tape, not the spec sheet. */
    public static final double WHEEL_DIAMETER_IN = 4.0;

    /** Centre-to-centre distance between the left and right wheels, in inches. */
    public static final double TRACK_WIDTH_IN = 12.0;

    // --- Motion -------------------------------------------------------------

    /** Forward power. Kept moderate so the wheels do not slip and lose counts. */
    public static final double DRIVE_POWER = 0.5;

    /** Turn power. Lower than the drive power -- a spin is easier to overshoot. */
    public static final double TURN_POWER = 0.4;

    /** How close to the target, in ticks, counts as "arrived". */
    public static final int TOLERANCE_TICKS = 15;

    /** Give up after this long with nothing reached, rather than driving forever. */
    public static final long FORWARD_TIMEOUT_MS = 8000;
    public static final long TURN_TIMEOUT_MS = 5000;

    private TankDrivebase drive;

    @Override
    public void runOpMode() {
        drive = new TankDrivebase(hardwareMap);

        // Convert inches and degrees into wheel ticks once, so the rest of the
        // OpMode reads as the two moves the request actually asked for.
        double ticksPerInch = TICKS_PER_WHEEL_REV / (Math.PI * WHEEL_DIAMETER_IN);

        int forwardTicks = (int) Math.round(SQUARES * TILE_IN * ticksPerInch);

        // A turn spins each side through the arc swept by a point at the track
        // radius: (TURN_DEG / 360) of a full circle of circumference pi * track.
        int turnTicks = (int) Math.round(
                (TURN_DEG / 360.0) * Math.PI * TRACK_WIDTH_IN * ticksPerInch);

        telemetry.addLine("Auto: 2 squares forward, then turn 72 deg clockwise.");
        telemetry.addLine("WHEELS OFF THE GROUND for the first run.");
        telemetry.addLine();
        telemetry.addData("forward", "%d ticks (%.1f in)",
                forwardTicks, SQUARES * TILE_IN);
        telemetry.addData("turn", "%d ticks (%.0f deg)", turnTicks, TURN_DEG);
        telemetry.addLine();
        telemetry.addLine("If these are wrong, the geometry constants at the");
        telemetry.addLine("top of ForwardAndTurnAuto.java need measuring.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean forwardOk = runMove("forward " + format(SQUARES) + " squares",
                forwardTicks, forwardTicks, DRIVE_POWER, FORWARD_TIMEOUT_MS);

        // A clockwise spin is left side forward, right side backward. driveToTicks
        // takes the direction from the sign of each target, not from the power.
        boolean turnOk = runMove("turn " + format(TURN_DEG) + " deg",
                (int) (TURN_SIGN * turnTicks), (int) (-TURN_SIGN * turnTicks),
                TURN_POWER, TURN_TIMEOUT_MS);

        drive.stop();

        telemetry.addData("forward", forwardOk ? "reached" : "TIMED OUT -- check geometry");
        telemetry.addData("turn", turnOk ? "reached" : "TIMED OUT -- check geometry");
        telemetry.addLine();
        telemetry.addLine("Done. If a move timed out, the robot may be stuck or the");
        telemetry.addLine("ticks-per-inch is too low -- see the header of this file.");
        telemetry.update();

        // Hold the final position so the robot does not roll after the move.
        sleep(2000);
    }

    /**
     * Run both sides to a relative tick target and wait until they get there, or
     * until the timeout. Returns true if the target was reached.
     *
     * The target is relative to wherever the encoders are now, so the caller thinks
     * in "move this far", not "drive to some absolute count". Both sides are stopped
     * on the way out, reached or not -- a half-finished move must not leave a wheel
     * turning while the next one starts.
     */
    private boolean runMove(String label, int leftDelta, int rightDelta,
                            double power, long timeoutMs) {
        int leftStart  = drive.leftTicks();
        int rightStart = drive.rightTicks();

        drive.driveToTicks(leftStart + leftDelta, rightStart + rightDelta, power);

        long begin = System.currentTimeMillis();
        while (opModeIsActive()) {
            int leftMoved  = drive.leftTicks()  - leftStart;
            int rightMoved = drive.rightTicks() - rightStart;

            telemetry.addData("move", label);
            telemetry.addData("  left",  "%d / %d ticks", leftMoved,  leftDelta);
            telemetry.addData("  right", "%d / %d ticks", rightMoved, rightDelta);
            telemetry.addData("  elapsed", "%d ms", System.currentTimeMillis() - begin);
            telemetry.update();

            boolean arrived = Math.abs(leftDelta  - leftMoved)  <= TOLERANCE_TICKS
                           && Math.abs(rightDelta - rightMoved) <= TOLERANCE_TICKS;
            if (arrived) {
                drive.stop();
                settle();
                return true;
            }
            if (System.currentTimeMillis() - begin > timeoutMs) {
                drive.stop();
                settle();
                return false;
            }
            idle();
        }

        drive.stop();
        return false;
    }

    /** Let the wheels actually come to rest before we read position or start again. */
    private void settle() {
        sleep(150);
    }

    /** Trim a trailing ".0" so "2.0" reads as "2" and "72.5" keeps its fraction. */
    private static String format(double value) {
        return (value == Math.floor(value))
                ? String.valueOf((long) value)
                : String.valueOf(value);
    }
}
