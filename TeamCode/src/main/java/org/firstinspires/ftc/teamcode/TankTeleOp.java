package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Tank TeleOp for the goBILDA StarterBot / kitbot -- one motor per side.
 *
 * This is the counterpart to {@link MecanumTeleOp}. It drives the same way by
 * default (left stick translate, right stick turn) so a driver can move between
 * the practice bot and the kitbot without relearning anything, but there is no
 * strafe and no field-centric mode: a skid-steer cannot slide sideways, and
 * "field-centric" only means anything when it can.
 *
 *   left stick      translate (forward / back)
 *   right stick x   turn
 *   right trigger   precision creep -- scales everything down for lining up
 *   back            toggle ARCADE steering / TANK steering
 *   right bumper    run the intake (hold to run; let go to stop)
 *   left bumper     reverse the intake -- clears a jam
 *   dpad up/down    raise / lower the lift (hold to run; let go to stop)
 *
 * ARCADE steering is the default: one stick drives, the other turns, and the
 * two are mixed. TANK steering drives each side from its own stick, which is
 * what a driver who learned on a two-stick setup expects -- left stick is the
 * left wheel, right stick is the right wheel.
 *
 * FIRST BRINGUP, in this order, on blocks with the wheels off the ground:
 *
 *  1. Push the left stick forward. BOTH wheels must spin forward. A side going
 *     backwards is a reversed motor, not a math problem -- flip LEFT_REVERSE or
 *     RIGHT_REVERSE in TankDrivebase.
 *  2. Push the left stick right. The robot must spin clockwise. If it spins the
 *     wrong way, swap the sign of the turn, not the motors.
 *  3. Only then set it on the floor.
 */
@TeleOp(name = "2. Tank TeleOp (kitbot)", group = "Drive")
public class TankTeleOp extends LinearOpMode {

    /**
     * Gamepad sticks rarely read exactly zero when you let go -- worn ones
     * especially. Anything smaller than this counts as "not touched", so the
     * robot sits still instead of creeping across the field on its own.
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
        TankDrivebase drive = new TankDrivebase(hardwareMap);
        AttachmentMotors attachments = new AttachmentMotors(hardwareMap);

        boolean tankSteering = false;
        boolean backWasPressed = false;

        telemetry.addLine("Ready. Point the robot downfield before START.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Gamepad y is negative when pushed forward; flip it so +forward is forward.
            double forward = deadband(-gamepad1.left_stick_y);
            double turn    = deadband(gamepad1.right_stick_x);

            // Multiplying a stick value by itself makes small pushes much gentler
            // while a full push still gives full power. That gives you fine control
            // near the middle of the stick, which is where careful lining-up happens.
            // (copySign puts the minus sign back, since a negative times a negative
            // would come out positive and you'd drive the wrong way.)
            forward = square(forward);
            turn    = square(turn);

            double scale = 1.0 - (1.0 - CREEP_SCALE) * gamepad1.right_trigger;
            forward *= scale;
            turn    *= scale;

            if (gamepad1.back && !backWasPressed) {
                tankSteering = !tankSteering;
            }
            backWasPressed = gamepad1.back;

            if (tankSteering) {
                // Each stick drives its own side. The right stick y is the right
                // side, so it needs the same forward-is-negative flip.
                double leftSide  = forward;
                double rightSide = square(deadband(-gamepad1.right_stick_y)) * scale;
                drive.driveTank(leftSide, rightSide);
            } else {
                drive.driveRobotCentric(forward, turn);
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

            telemetry.addData("steering", tankSteering ? "TANK (each stick a side)"
                                                        : "ARCADE (stick + turn)");
            telemetry.addData("stick", "fwd %+.2f  turn %+.2f", forward, turn);
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
