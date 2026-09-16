package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The attachment motors: one intake and two lift motors.
 *
 * Robot Configuration names expected on the Control Hub:
 *
 *     intake        lift_left        lift_right
 *
 * These are simple open-loop spin motors -- no encoders, no holding a position.
 * If a lift later needs to hold against gravity or move a measured distance, that
 * is a different class; this one deliberately does not pretend to.
 *
 * MISSING MOTORS DO NOT CRASH THE ROBOT. Each one is looked up with tryGet, so if
 * the configuration does not have them yet the drivebase still runs and this class
 * quietly does nothing. That is on purpose: the drivebase had to work before the
 * attachments existed, and it should keep working while a mechanism is being built.
 * HardwareCheck is where you find out a name is wrong.
 *
 * A LIFT MOTOR THAT SPINS THE WRONG WAY is a reversed motor, not a math problem.
 * The two lift motors sit on opposite sides of a shared mechanism, so one of them
 * usually has to be reversed -- but which one depends on how yours is built, so
 * both default forward and the two constants below are the one-line fix. Test on
 * blocks BEFORE the lift can hit anything.
 */
public class AttachmentMotors {

    public static final String INTAKE     = "intake";
    public static final String LIFT_LEFT  = "lift_left";
    public static final String LIFT_RIGHT = "lift_right";

    /**
     * Flip these if a lift motor fights the other one, or if up is down.
     * Changing which is reversed is a code fix; changing ports is a build fix.
     */
    private static final boolean LIFT_LEFT_REVERSE  = false;
    private static final boolean LIFT_RIGHT_REVERSE = false;

    /**
     * Strength multipliers, so a mechanism can be slowed down without editing the
     * buttons. 1.0 is full power.
     */
    public static final double INTAKE_GAIN = 1.0;
    public static final double LIFT_GAIN   = 1.0;

    private final DcMotor intake;
    private final DcMotor liftLeft;
    private final DcMotor liftRight;

    public AttachmentMotors(HardwareMap hardwareMap) {
        intake    = hardwareMap.tryGet(DcMotor.class, INTAKE);
        liftLeft  = hardwareMap.tryGet(DcMotor.class, LIFT_LEFT);
        liftRight = hardwareMap.tryGet(DcMotor.class, LIFT_RIGHT);

        if (intake != null) {
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            // BRAKE so a stalled intake does not freewheel once you let go.
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (DcMotor m : new DcMotor[] { liftLeft, liftRight }) {
            if (m == null) continue;
            // BRAKE is the safe default on a lift: a lift that coasts down under
            // gravity is a mechanism that drops when you release the button.
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (liftLeft  != null) liftLeft.setDirection(LIFT_LEFT_REVERSE
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (liftRight != null) liftRight.setDirection(LIFT_RIGHT_REVERSE
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    /** +1 pulls in, -1 pushes out. Guarded so a missing motor is harmless. */
    public void setIntake(double power) {
        if (intake == null) return;
        intake.setPower(clamp(power) * INTAKE_GAIN);
    }

    public void stopIntake() {
        setIntake(0);
    }

    /** +1 raises, -1 lowers. Drives both lift motors together. */
    public void setLift(double power) {
        double p = clamp(power) * LIFT_GAIN;
        if (liftLeft  != null) liftLeft.setPower(p);
        if (liftRight != null) liftRight.setPower(p);
    }

    public void stopLift() {
        setLift(0);
    }

    /** Both mechanisms off. Call this when the OpMode ends. */
    public void stop() {
        stopIntake();
        stopLift();
    }

    private static double clamp(double v) {
        if (v > 1.0) return 1.0;
        if (v < -1.0) return -1.0;
        return v;
    }
}
