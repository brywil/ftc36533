package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Four-motor mecanum drivebase.
 *
 * Robot Configuration names expected on the Control Hub:
 *
 *     front_left   front_right   back_left   back_right
 *
 * and, for field-centric driving, the built-in "imu" on the Control Hub's I2C bus 0.
 *
 * The motors on the left are told to run backwards. That sounds odd, but the left
 * and right motors face opposite ways on the robot, so "forwards" for one is
 * "backwards" for the other. This is correct for the normal goBILDA/REV build.
 *
 * If a wheel spins the wrong way on your robot, fix it here -- but test on blocks
 * first. GETTING_STARTED.md, Part 3, walks through how to check.
 */
public class MecanumDrivebase {

    /**
     * Sliding sideways is weaker than driving forwards, even when the motors are
     * working just as hard. The angled rollers waste some of the push. This number
     * gives sideways a boost to even things up.
     *
     * To tune it: lay tape on the floor in a square, drive around it, and if the
     * sideways sides come out short, raise this a little.
     */
    public static final double STRAFE_GAIN = 1.1;

    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;

    /** Offset subtracted from the IMU yaw, so the driver can re-zero "away from me". */
    private double headingOffset = 0.0;

    public MecanumDrivebase(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left");
        backRight  = hardwareMap.get(DcMotor.class, "back_right");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // BRAKE makes the motors stop quickly when you let go of the stick. The
        // other option, FLOAT, lets the robot coast -- smoother, but it drifts, and
        // drifting off a scoring position loses points.
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (DcMotor m : new DcMotor[] { frontLeft, frontRight, backLeft, backRight }) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
    }

    /**
     * Drive using the robot's own directions: +forward is where the nose points,
     * +strafe is to its right, +turn spins it clockwise seen from above. All three
     * numbers run from -1 to +1.
     *
     * The "denominator" line below is the important part. If you push the stick all
     * the way diagonally AND turn at the same time, the maths can ask a motor for
     * more power than it actually has. The motor just gives what it can, and the
     * robot curves off somewhere you did not ask for. Dividing all four by the
     * biggest request keeps them in proportion -- so the robot goes exactly where
     * you pointed, just a little slower.
     */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        strafe *= STRAFE_GAIN;

        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);

        setPowers(
                (forward + strafe + turn) / denominator,   // front left
                (forward - strafe - turn) / denominator,   // front right
                (forward - strafe + turn) / denominator,   // back left
                (forward + strafe - turn) / denominator);  // back right
    }

    /**
     * Drive using the FIELD's directions: pushing the stick away from you always
     * sends the robot away from you, even if it has spun around. Most drivers find
     * this much easier than thinking about which way the nose points.
     *
     * This only works if the robot knows which way it is facing, which it learns
     * from the IMU. Point the robot away from the drivers and call resetHeading()
     * before the match, or "away" will mean the wrong direction.
     */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        double heading = getHeading();
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        double rotStrafe  = strafe * cos - forward * sin;
        double rotForward = strafe * sin + forward * cos;

        driveRobotCentric(rotForward, rotStrafe, turn);
    }

    public void setPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    /** Field-frame heading in radians, CCW positive, with the driver's offset applied. */
    public double getHeading() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return normalize(yaw - headingOffset);
    }

    /** Call with the robot pointed the way the driver considers "forward". */
    public void resetHeading() {
        imu.resetYaw();
        headingOffset = 0.0;
    }

    private static double normalize(double radians) {
        while (radians > Math.PI)  radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
