package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Basic mecanum TeleOp.
 *
 *   left stick      translate (forward / strafe)
 *   right stick x   turn
 *   right trigger   precision creep -- scales everything down for lining up
 *   options         re-zero the field-centric heading to wherever the robot points
 *   back            toggle field-centric / robot-centric
 *   right bumper    run the intake (hold to run; let go to stop)
 *   left bumper     reverse the intake -- clears a jam
 *   dpad up/down    raise / lower the lift (hold to run; let go to stop)
 *
 * FIRST BRINGUP, in this order, on blocks with the wheels off the ground:
 *
 *  1. Push the left stick forward. All four wheels must spin forward. A wheel
 *     spinning backwards is a reversed motor, not a math problem -- fix the
 *     direction in MecanumDrivebase.
 *  2. Push the left stick right. The wheels must form an X pattern seen from
 *     above (front-left and back-right spinning forward). If it rotates instead,
 *     two motors are swapped in the Robot Configuration.
 *  3. Only then set it on the floor. A mecanum chassis with one roller set
 *     mounted backwards drives fine forward and crabs sideways on every turn.
 */
@TeleOp(name = "2. Mecanum TeleOp (field-centric)", group = "Drive")
public class MecanumTeleOp extends LinearOpMode {

    /**
     * Gamepad sticks rarely read exactly zero when you let go -- worn ones especially.
     * Anything smaller than this counts as "not touched", so the robot sits still
     * instead of creeping across the field on its own.
     */
    private static final double DEADBAND = 0.05;

    /** Multiplier at full precision trigger. */
    private static final double CREEP_SCALE = 0.30;

    /** Intake power. Right bumper runs it at this; left bumper runs it reversed. */
    private static final double INTAKE_POWER = 1.0;

    /** Lift power while a dpad direction is held. Kept below 1 for a first test. */
    private static final double LIFT_POWER = 0.6;

    @Override
    public void runOpMode() {
        MecanumDrivebase drive = new MecanumDrivebase(hardwareMap);
        AttachmentMotors attachments = new AttachmentMotors(hardwareMap);

        boolean fieldCentric = true;
        boolean backWasPressed = false;

        telemetry.addLine("Ready. Point the robot downfield before START.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
        drive.resetHeading();

        while (opModeIsActive()) {
            // Gamepad y is negative when pushed forward; flip it so +forward is forward.
            double forward = deadband(-gamepad1.left_stick_y);
            double strafe  = deadband(gamepad1.left_stick_x);
            double turn    = deadband(gamepad1.right_stick_x);

            // Multiplying the stick value by itself makes small pushes much gentler
            // while a full push still gives full power. That gives you fine control
            // near the middle of the stick, which is where careful lining-up happens.
            // (copySign puts the minus sign back, since a negative times a negative
            // would come out positive and you'd drive the wrong way.)
            forward = square(forward);
            strafe  = square(strafe);
            turn    = square(turn);

            double scale = 1.0 - (1.0 - CREEP_SCALE) * gamepad1.right_trigger;
            forward *= scale;
            strafe  *= scale;
            turn    *= scale;

            if (gamepad1.options) {
                drive.resetHeading();
            }
            if (gamepad1.back && !backWasPressed) {
                fieldCentric = !fieldCentric;
            }
            backWasPressed = gamepad1.back;

            if (fieldCentric) {
                drive.driveFieldCentric(forward, strafe, turn);
            } else {
                drive.driveRobotCentric(forward, strafe, turn);
            }

            // Attachments. Both are hold-to-run, so nothing keeps spinning after a
            // mechanism jams or a ball is where it should be -- the safe failure is
            // always "let go". Reverse on the left bumper clears a jam in the intake.
            double intakePower = 0.0;
            if (gamepad1.right_bumper) intakePower = INTAKE_POWER;
            else if (gamepad1.left_bumper) intakePower = -INTAKE_POWER;
            attachments.setIntake(intakePower);

            double liftPower = 0.0;
            if (gamepad1.dpad_up) liftPower = LIFT_POWER;
            else if (gamepad1.dpad_down) liftPower = -LIFT_POWER;
            attachments.setLift(liftPower);

            telemetry.addData("mode", fieldCentric ? "FIELD-centric" : "ROBOT-centric");
            telemetry.addData("heading", "%.1f deg", Math.toDegrees(drive.getHeading()));
            telemetry.addData("stick", "fwd %+.2f  str %+.2f  turn %+.2f",
                    forward, strafe, turn);
            telemetry.addData("intake", "%+.2f  (RB in, LB reverse)", intakePower);
            telemetry.addData("lift", "%+.2f  (dpad up/down)", liftPower);
            telemetry.update();
        }

        drive.stop();
        attachments.stop();
    }

    private static double deadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : value;
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }
}
