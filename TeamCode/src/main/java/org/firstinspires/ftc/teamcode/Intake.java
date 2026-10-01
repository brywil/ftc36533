package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The intake: one roller motor and two continuous-rotation servos that pull POLLEN in.
 *
 * Robot Configuration names expected on the Control Hub:
 *
 *     intake        left_intake_servo        right_intake_servo
 *
 * These match goBILDA's own BIOBUZZ StarterBot example code, so one Robot Configuration
 * drives both their sample and this code. The two side servos help pull elements out of
 * corners, where the roller alone cannot reach.
 *
 * ONE POWER FOR ALL THREE. They are a single mechanism as far as the driver is
 * concerned -- push the stick (or hold the bumper) and balls go in; release and they
 * stop. So this class takes one power and applies it to the motor and both servos,
 * rather than exposing three separate controls. A CR servo takes power exactly like a
 * motor (-1..1), so the same number drives all three.
 *
 * The right servo is reversed, because it sits on the opposite side of the intake and
 * has to spin the other way to pull in the same direction -- goBILDA's kit reverses it,
 * so this does too. That is a build fact, not a bug: flip RIGHT_SERVO_REVERSE if your
 * wiring differs.
 *
 * NO LIFTS. This robot has no lift mechanism; earlier code here drove two lift motors
 * that do not exist on the StarterBot and were silently inert. They are gone.
 *
 * MISSING PARTS DO NOT CRASH THE ROBOT. Each is looked up with tryGet, so a robot
 * without the servos still runs the roller, and HardwareCheck names what is absent.
 */
public class Intake {

    public static final String MOTOR       = "intake";
    public static final String LEFT_SERVO  = "left_intake_servo";
    public static final String RIGHT_SERVO = "right_intake_servo";

    /** The right servo sits on the far side of the intake and pulls the other way. */
    private static final boolean RIGHT_SERVO_REVERSE = true;

    private final DcMotor motor;
    private final CRServo leftServo;
    private final CRServo rightServo;

    public Intake(HardwareMap hardwareMap) {
        motor     = hardwareMap.tryGet(DcMotor.class, MOTOR);
        leftServo = hardwareMap.tryGet(CRServo.class, LEFT_SERVO);
        rightServo = hardwareMap.tryGet(CRServo.class, RIGHT_SERVO);

        if (motor != null) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            // BRAKE so a stalled intake does not freewheel once you let go.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (leftServo != null) {
            leftServo.setDirection(DcMotorSimple.Direction.FORWARD);
            leftServo.setPower(0);
        }
        if (rightServo != null) {
            rightServo.setDirection(RIGHT_SERVO_REVERSE
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            rightServo.setPower(0);
        }
    }

    public boolean hasMotor()  { return motor != null; }
    public boolean hasServos() { return leftServo != null || rightServo != null; }

    /** +1 pulls in, -1 pushes out. Drives the roller and both servos together. */
    public void setPower(double power) {
        setMotor(power);
        setServos(power);
    }

    /**
     * The roller motor alone. Used for the reverse/clear-jam control, which must not
     * turn the servos -- reversing the side servos would fight the roller.
     */
    public void setMotor(double power) {
        if (motor != null) motor.setPower(clamp(power));
    }

    /** Both corner servos together, at their own power. */
    public void setServos(double power) {
        double p = clamp(power);
        if (leftServo != null)  leftServo.setPower(p);
        if (rightServo != null) rightServo.setPower(p);
    }

    public void stop() {
        setMotor(0);
        setServos(0);
    }

    private static double clamp(double v) {
        if (v > 1.0) return 1.0;
        if (v < -1.0) return -1.0;
        return v;
    }
}
