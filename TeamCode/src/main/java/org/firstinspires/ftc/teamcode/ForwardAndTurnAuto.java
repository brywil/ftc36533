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
 * Dead reckoning multiplies errors, so these are the whole story. All three were
 * measured on this robot (2026-10-07); re-measure if the drive changes:
 *
 *   TICKS_PER_WHEEL_REV  the motor's encoder counts per OUTPUT-shaft revolution.
 *                        537.7 = 28 counts per motor turn x the 19.2:1 gearbox.
 *                        Measured by hand as 2683 ticks over 5 wheel turns = 536.6.
 *                        Getting this wrong scales every distance and angle by the
 *                        same factor.
 *   WHEEL_DIAMETER_IN    the driven wheel's real diameter, tape-measured at 3.75 in.
 *                        (The 96 mm goBILDA wheel is nominally 3.78 in, often
 *                        quoted as "4".)
 *   TRACK_WIDTH_IN       the distance between the centres of the left and right
 *                        wheels, tape-measured at 15.75 in. It is what turns wheel
 *                        travel into a rotation angle. Wrong track width is why a
 *                        "72 degree" turn comes out as 65 or 80.
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
 * on a timeout rather than running away, and says so on the Driver Station. If a
 * side's encoder stops counting altogether, the move is stopped early instead of
 * waiting out the timeout, because no amount of driving will ever "arrive".
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

    /**
     * Encoder counts per output-shaft revolution. 537.7 = 28 counts per motor turn
     * x the 19.2:1 gearbox (measured by hand: 2683 ticks over 5 wheel turns). The
     * old comment here said 5.23:1, which would be ~145, not 537.7.
     */
    public static final double TICKS_PER_WHEEL_REV = 537.7;

    /** Driven wheel diameter in inches, tape-measured across the tread. */
    public static final double WHEEL_DIAMETER_IN = 3.75;

    /** Centre-to-centre distance between the left and right wheels, tape-measured. */
    public static final double TRACK_WIDTH_IN = 15.75;

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

    /**
     * If neither side's encoder has counted a tick by this long into a move, treat
     * the encoders as dead (usually an unplugged cable or the wrong port) and stop.
     * Set above the motor's own startup lag so a normal move is not cut short.
     */
    public static final long ENCODER_STARTUP_GRACE_MS = 500;

    /** The status string a move returns when it reached its target. */
    private static final String REACHED = "reached";

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

        String forwardResult = runMove("forward " + format(SQUARES) + " squares",
                forwardTicks, forwardTicks, DRIVE_POWER, FORWARD_TIMEOUT_MS);

        // A clockwise spin is left side forward, right side backward. driveToTicks
        // takes the direction from the sign of each target, not from the power.
        String turnResult = runMove("turn " + format(TURN_DEG) + " deg",
                (int) (TURN_SIGN * turnTicks), (int) (-TURN_SIGN * turnTicks),
                TURN_POWER, TURN_TIMEOUT_MS);

        drive.stop();

        telemetry.addData("forward", forwardResult);
        telemetry.addData("turn", turnResult);
        telemetry.addLine();
        telemetry.addLine("Done. If a move timed out, the robot may be stuck or the");
        telemetry.addLine("ticks-per-inch is too low -- see the header of this file.");
        telemetry.addLine("If it stopped on an encoder warning, check those cables.");
        telemetry.update();

        // Leave the telemetry up for a moment so the result can be read on the
        // Driver Station. stop() only brakes; nothing actively holds the robot.
        sleep(2000);
    }

    /**
     * Run both sides to a relative tick target and wait until they get there, or
     * until the timeout or an encoder failure. Returns a status string for the
     * Driver Station: "reached", or why it stopped early.
     *
     * The target is relative to wherever the encoders are now, so the caller thinks
     * in "move this far", not "drive to some absolute count". Both sides are stopped
     * on the way out, reached or not -- a half-finished move must not leave a wheel
     * turning while the next one starts.
     */
    private String runMove(String label, int leftDelta, int rightDelta,
                           double power, long timeoutMs) {
        int leftStart  = drive.leftTicks();
        int rightStart = drive.rightTicks();

        drive.driveToTicks(leftStart + leftDelta, rightStart + rightDelta, power);

        long begin = System.currentTimeMillis();
        while (opModeIsActive()) {
            int leftMoved  = drive.leftTicks()  - leftStart;
            int rightMoved = drive.rightTicks() - rightStart;
            long elapsed   = System.currentTimeMillis() - begin;

            // A side that was asked to move but has not counted a single tick after
            // the grace period has a cable problem, not a slow robot. Check each side
            // on its own: one missing cable lets the other side run on, so the robot
            // curves (straight move) or spins until the timeout (turn) -- a runaway
            // either way. No amount of driving will satisfy a dead encoder.
            boolean leftDead  = leftDelta  != 0 && leftMoved  == 0;
            boolean rightDead = rightDelta != 0 && rightMoved == 0;
            boolean encodersDead = elapsed > ENCODER_STARTUP_GRACE_MS
                                && (leftDead || rightDead);

            telemetry.addData("move", label);
            telemetry.addData("  left",  "%d / %d ticks%s", leftMoved,  leftDelta,
                    leftDead ? "   <-- no encoder reading" : "");
            telemetry.addData("  right", "%d / %d ticks%s", rightMoved, rightDelta,
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

            boolean arrived = Math.abs(leftDelta  - leftMoved)  <= TOLERANCE_TICKS
                           && Math.abs(rightDelta - rightMoved) <= TOLERANCE_TICKS;
            if (arrived) {
                drive.stop();
                settle();
                return REACHED;
            }
            if (elapsed > timeoutMs) {
                drive.stop();
                settle();
                return "TIMED OUT -- check geometry";
            }
            idle();
        }

        drive.stop();
        return "stopped";
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
