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
 * Wiring convention: this class reverses the LEFT side, which is correct when all
 * four motors are mounted with their output shafts pointing outward from the chassis
 * (the usual goBILDA/REV layout). If your robot strafes when you push forward, you
 * have a direction wrong -- see the check in the class comment of MecanumTeleOp.
 */
public class MecanumDrivebase {

    /**
     * Mecanum rollers make strafing weaker than forward travel for the same motor
     * power -- friction and the 45-degree roller contact both work against it. This
     * scales the strafe axis up to compensate. Tune it by driving a taped square:
     * raise it if the square comes out short in the strafe direction.
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

        // BRAKE keeps the robot where the driver left it. FLOAT coasts, which feels
        // smoother but drifts on a bumpy field.
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
     * Drive in the robot's own frame: +forward is the robot's nose, +strafe is its
     * right, +turn is clockwise seen from above.
     *
     * All three inputs are -1..1. The denominator normalises them together, so a
     * full-stick diagonal keeps its direction instead of clipping into a curve.
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
     * Drive in the FIELD's frame: +forward is away from the driver station no matter
     * which way the robot is pointing. Depends entirely on the IMU heading, so call
     * {@link #resetHeading()} with the robot aimed downfield before the match.
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
