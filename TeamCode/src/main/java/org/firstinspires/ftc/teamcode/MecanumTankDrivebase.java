package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A mecanum chassis driven as a tank: four motors, two sides, no strafing.
 *
 * A mecanum robot can slide sideways because each of its four wheels is driven
 * independently. This class deliberately throws that away: the two left motors get
 * one command, the two right motors get one command, and the only way to turn is to
 * run the sides at different speeds -- exactly like a two-motor skid-steer.
 *
 * WHY NOT JUST USE {@link MecanumDrivebase}. That class is the full mecanum base:
 * strafe, field-centric mode, and it requires the IMU to be in the Robot
 * Configuration. A tank driver needs none of that, and a new chassis may not have
 * the IMU configured yet. This class owns only the drive motors.
 *
 * MOTOR NAMES. It accepts either naming scheme, so the same OpMode works whichever
 * way the Robot Configuration was typed:
 *
 *     front_left  front_right  back_left  back_right
 *     left_front_drive  right_front_drive  left_back_drive  right_back_drive
 *
 * If only two drive motors are configured it falls back to {@code left_drive} /
 * {@code right_drive} (the goBILDA kitbot), so a two-motor robot is not left out.
 *
 * MOTOR DIRECTIONS. The left motors are REVERSE and the right motors FORWARD for the
 * four-wheel layout (the same convention {@link MecanumDrivebase} uses); the two-motor
 * fallback uses {@link TankDrivebase}'s convention (left FORWARD, right REVERSE).
 * Those are the normal builds. If one side drives the wrong way on your robot, flip
 * the directions HERE, not in the OpMode, so every program stays in agreement.
 */
public class MecanumTankDrivebase {

    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final boolean fourWheel;
    private final boolean ready;

    /** What was last asked of each side, -1..+1. Kept for telemetry. */
    private double lastLeft, lastRight;

    public MecanumTankDrivebase(HardwareMap hardwareMap) {
        DcMotor fl = firstOf(hardwareMap, "front_left",  "left_front_drive");
        DcMotor fr = firstOf(hardwareMap, "front_right", "right_front_drive");
        DcMotor bl = firstOf(hardwareMap, "back_left",   "left_back_drive");
        DcMotor br = firstOf(hardwareMap, "back_right",  "right_back_drive");

        boolean four = fl != null && fr != null && bl != null && br != null;

        if (four) {
            fl.setDirection(DcMotorSimple.Direction.REVERSE);
            bl.setDirection(DcMotorSimple.Direction.REVERSE);
            fr.setDirection(DcMotorSimple.Direction.FORWARD);
            br.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            // Not a complete four-motor set: try the two-motor kitbot names.
            fl = firstOf(hardwareMap, "left_drive", "left");
            fr = firstOf(hardwareMap, "right_drive", "right");
            bl = null;
            br = null;
            if (fl != null) fl.setDirection(DcMotorSimple.Direction.FORWARD);
            if (fr != null) fr.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
        fourWheel = four;
        ready = frontLeft != null && frontRight != null;

        for (DcMotor m : new DcMotor[] { frontLeft, frontRight, backLeft, backRight }) {
            if (m == null) continue;
            // BRAKE: stop when the stick is released instead of coasting past the
            // thing the driver was lining up on.
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** True when at least one motor is on each side; false means only "no motors". */
    public boolean isReady() { return ready; }

    public boolean isFourWheel() { return fourWheel; }

    /** One line for the Driver Station naming exactly what was found. */
    public String describe() {
        if (!ready) return "NO DRIVE MOTORS FOUND";
        if (fourWheel) return "4 motors, driven as two sides";
        return "2 motors (left_drive / right_drive)";
    }

    /**
     * Arcade tank driving: +forward drives both sides forward, +turn spins clockwise
     * seen from above. The denominator keeps a full-speed forward plus a full-speed
     * turn from asking a side for more power than it has.
     */
    public void driveArcade(double forward, double turn) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(turn), 1.0);
        setSides((forward + turn) / denominator,
                 (forward - turn) / denominator);
    }

    /**
     * True tank steering: each number drives its own side directly. +left/+right both
     * drive that side forward; +left with -right spins clockwise.
     */
    public void driveTank(double left, double right) {
        setSides(left, right);
    }

    private void setSides(double left, double right) {
        lastLeft = clamp(left);
        lastRight = clamp(right);
        if (frontLeft != null)  frontLeft.setPower(lastLeft);
        if (backLeft != null)   backLeft.setPower(lastLeft);
        if (frontRight != null) frontRight.setPower(lastRight);
        if (backRight != null)  backRight.setPower(lastRight);
    }

    public void stop() { setSides(0.0, 0.0); }

    /** The powers last sent to each side, for telemetry. */
    public double leftPower() { return lastLeft; }
    public double rightPower() { return lastRight; }

    private static DcMotor firstOf(HardwareMap hardwareMap, String... names) {
        for (String name : names) {
            DcMotor motor = hardwareMap.tryGet(DcMotor.class, name);
            if (motor != null) return motor;
        }
        return null;
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}
