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
 * Shooting rides on the driver's spare buttons, the same layout as TankTeleOp, so a
 * match driver can score without switching OpModes:
 *
 *   X               arm / disarm the flywheel
 *   Y               AUTO (from the tag distance) / MANUAL (fixed power)
 *   left trigger    feed a ball (hold) -- only works once the wheel is up to speed
 *   B               reverse the feed -- clears a jam
 *   dpad left/right manual power -/+ (when in MANUAL)
 *
 * If the Robot Configuration has no "limelight", AUTO cannot answer, so the shooter
 * starts in MANUAL. If it has no "flywheel" motor, the shooter does nothing and the
 * robot drives exactly as before.
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

    /** Manual power step for the dpad-left/right tuning buttons. */
    private static final double MANUAL_STEP = 0.02;

    /** The Limelight pipeline configured for AprilTags: the ball detector is 0. */
    private static final int FIDUCIAL_PIPELINE = 1;

    @Override
    public void runOpMode() {
        MecanumDrivebase drive = new MecanumDrivebase(hardwareMap);
        AttachmentMotors attachments = new AttachmentMotors(hardwareMap);

        ShooterControls shooter = new ShooterControls(new Shooter(hardwareMap));
        LimelightHiveTracker tracker =
                new LimelightHiveTracker(hardwareMap, "limelight", FIDUCIAL_PIPELINE);
        // No camera means AUTO can never answer; fall back to MANUAL up front rather
        // than showing an AUTO that silently never fires.
        if (!tracker.hasLimelight()) shooter.fallBackToManual();

        boolean fieldCentric = true;
        boolean backWasPressed = false;
        boolean xPrev = false, yPrev = false;
        boolean dpadLeftPrev = false, dpadRightPrev = false;

        telemetry.addLine("Ready. Point the robot downfield before START.");
        telemetry.addData("limelight", tracker.hasLimelight() ? "OK" : "MISSING (shooter = MANUAL)");
        telemetry.addData("flywheel", shooter.shooter().hasFlywheel() ? "OK" : "MISSING");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
        drive.resetHeading();
        tracker.start();

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

            // Shooter on the driver's spare buttons, same layout as TankTeleOp.
            if (gamepad1.x && !xPrev) shooter.toggleArm();
            if (gamepad1.y && !yPrev) shooter.toggleManual();
            xPrev = gamepad1.x; yPrev = gamepad1.y;

            if (shooter.isManual()) {
                if (gamepad1.dpad_right && !dpadRightPrev) shooter.nudgeManualPower(+MANUAL_STEP);
                if (gamepad1.dpad_left  && !dpadLeftPrev)  shooter.nudgeManualPower(-MANUAL_STEP);
            }
            dpadLeftPrev = gamepad1.dpad_left; dpadRightPrev = gamepad1.dpad_right;

            // The camera is only asked for a range when armed in AUTO.
            Double rangeIn = (shooter.isArmed() && !shooter.isManual())
                    ? tracker.nearestRangeIn() : null;
            shooter.update(rangeIn, System.currentTimeMillis());
            shooter.requestFeed(gamepad1.left_trigger > 0.5, gamepad1.b);

            telemetry.addData("mode", fieldCentric ? "FIELD-centric" : "ROBOT-centric");
            telemetry.addData("heading", "%.1f deg", Math.toDegrees(drive.getHeading()));
            telemetry.addData("stick", "fwd %+.2f  str %+.2f  turn %+.2f",
                    forward, strafe, turn);
            telemetry.addData("intake", "%+.2f  (RB in, LB reverse)", intakePower);
            telemetry.addData("lift", "%+.2f  (dpad up/down)", liftPower);
            telemetry.addData("shooter", "%s  power %.2f  %s",
                    shooter.powerSource(), shooter.getCommandedPower(), shooter.feedState());
            if (!Double.isNaN(shooter.getLastRangeIn())) {
                telemetry.addData("  range", "%.1f in", shooter.getLastRangeIn());
            }
            telemetry.addLine("X arm  Y auto/manual  LT feed  B clear  dpad-LR power");
            telemetry.update();
        }

        drive.stop();
        attachments.stop();
        shooter.stop();
        tracker.close();
    }

    private static double deadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : value;
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }
}
