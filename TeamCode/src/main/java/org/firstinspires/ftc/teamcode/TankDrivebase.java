package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Two-motor skid-steer drivebase -- the goBILDA StarterBot / kitbot.
 *
 * This is the tank counterpart to {@link MecanumDrivebase}: one motor drives the
 * whole left side, one drives the whole right side, and the only way to turn is
 * to run the two sides at different speeds. There is no sideways motion, so
 * there is no strafe input and no field-centric mode -- those need the wheels to
 * push in a direction the robot is not facing, which a skid-steer cannot do.
 *
 * Robot Configuration names expected on the Control Hub:
 *
 *     left_drive        right_drive
 *
 * These are goBILDA's names from the official 2026-27 BIOBUZZ StarterBot example
 * code, so the stock kit needs no renaming.
 *
 * The two sides face opposite ways on the robot, so one of them has to be told
 * to run backwards for "forward" to mean forward. goBILDA's kitbot has the left
 * motor forward and the right motor reversed; if your robot is built the other
 * way round, flip the two constants below rather than rewiring. Test on blocks
 * first -- GETTING_STARTED.md, Part 3.
 */
public class TankDrivebase {

    /** Robot Configuration names, matching goBILDA's StarterBot. */
    public static final String LEFT_MOTOR  = "left_drive";
    public static final String RIGHT_MOTOR = "right_drive";

    /**
     * Flip either of these if a side drives the wrong way. Changing one is a
     * code fix; changing which port a motor is in is a build fix.
     */
    private static final boolean LEFT_REVERSE  = false;
    private static final boolean RIGHT_REVERSE = true;

    private final DcMotor left, right;

    public TankDrivebase(HardwareMap hardwareMap) {
        left  = hardwareMap.get(DcMotor.class, LEFT_MOTOR);
        right = hardwareMap.get(DcMotor.class, RIGHT_MOTOR);

        left.setDirection(LEFT_REVERSE
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setDirection(RIGHT_REVERSE
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // BRAKE so the robot stops quickly when you let go of the stick instead
        // of coasting past the thing you were lining up on.
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (DcMotor m : new DcMotor[] { left, right }) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Drive using the robot's own directions: +forward is where the nose points,
     * +turn spins it clockwise seen from above. Both run from -1 to +1.
     *
     * "Arcade" style: one input drives both sides the same way and the other
     * turns. The "denominator" line is the important part -- a full-speed
     * forward plus a full-speed turn asks a side for more power than it has, and
     * the motor just gives what it can, so the robot curves off somewhere you
     * did not ask for. Dividing both sides by the biggest request keeps them in
     * proportion, so the robot goes exactly where you pointed, just a little
     * slower.
     */
    public void driveRobotCentric(double forward, double turn) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(turn), 1.0);
        setPowers((forward + turn) / denominator,
                  (forward - turn) / denominator);
    }

    /**
     * True tank steering: each input drives its own side directly. +left/+right
     * both drive that side forward; to spin clockwise, +left and -right. This is
     * the control scheme drivers who learned on a two-stick setup expect.
     */
    public void driveTank(double leftPower, double rightPower) {
        setPowers(leftPower, rightPower);
    }

    public void setPowers(double l, double r) {
        left.setPower(l);
        right.setPower(r);
    }

    public void stop() {
        setPowers(0, 0);
    }

    /**
     * Drive each side to an encoder count and stop there. This is the closed-loop
     * move that autonomous is built from: give the left side a distance in ticks
     * and the right side its own, and the motors run until each has covered it.
     *
     * The counts are in the same logical frame as {@link #driveRobotCentric}: +ticks
     * means "this side forward", whichever motor was reversed in the Robot
     * Configuration. So a straight move is (+t, +t), and a clockwise turn is
     * (+t, -t) -- the caller, not this class, owns the geometry that turns inches
     * and degrees into ticks.
     *
     * power is the magnitude to run at. Run-to-position drives toward the target
     * from whichever side the motor is on, so the sign of power does not pick the
     * direction -- the targets do.
     */
    public void driveToTicks(int leftTarget, int rightTarget, double power) {
        left.setTargetPosition(leftTarget);
        right.setTargetPosition(rightTarget);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(Math.abs(power));
        right.setPower(Math.abs(power));
    }

    /** The left side's encoder count, in the logical frame described above. */
    public int leftTicks() {
        return left.getCurrentPosition();
    }

    /** The right side's encoder count, in the logical frame described above. */
    public int rightTicks() {
        return right.getCurrentPosition();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        left.setZeroPowerBehavior(behavior);
        right.setZeroPowerBehavior(behavior);
    }
}
