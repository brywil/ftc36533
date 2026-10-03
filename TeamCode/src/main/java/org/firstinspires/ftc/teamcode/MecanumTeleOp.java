package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

/**
 * Basic mecanum TeleOp, for the four-wheel practice bot.
 *
 *   left stick      translate (forward / strafe)
 *   right stick x   turn
 *   right trigger   precision creep -- scales everything down for lining up
 *   options         re-zero the field-centric heading to wherever the robot points
 *   back            toggle field-centric / robot-centric
 *   right bumper    intake in (hold);  left bumper  intake out -- clears a jam
 *   X               arm / disarm the launcher
 *   Y               AUTO (from the tag distance) / MANUAL (fixed velocity)
 *   A               feed a ball (hold) -- only once the wheel is up to speed
 *   B               reverse the feed -- clears a jam
 *   dpad up/down    manual velocity +/- (when in MANUAL)
 *
 * The same intake/launcher/feed layout as TankTeleOp, so a driver can move between
 * robots without relearning. Names match goBILDA's BIOBUZZ example; see {@link Intake}
 * and {@link Shooter}.
 *
 * If the Robot Configuration has no "limelight", AUTO cannot answer, so the shooter
 * starts in MANUAL. If it has no "launcher", the shooter does nothing and the robot
 * drives exactly as before.
 *
 * FIRST BRINGUP, in this order, on blocks with the wheels off the ground:
 *
 *  1. Push the left stick forward. All four wheels must spin forward. A wheel spinning
 *     backwards is a reversed motor -- fix the direction in MecanumDrivebase.
 *  2. Push the left stick right. The wheels must form an X pattern seen from above. If
 *     it rotates instead, two motors are swapped in the Robot Configuration.
 *  3. Only then set it on the floor.
 */
@Disabled
@TeleOp(name = "2. Mecanum TeleOp (field-centric)", group = "Drive")
public class MecanumTeleOp extends LinearOpMode {

    /** Gamepad sticks rarely read exactly zero when you let go. Below this is "not touched". */
    private static final double DEADBAND = 0.05;

    /** Multiplier at full precision trigger. */
    private static final double CREEP_SCALE = 0.30;

    /** The Limelight pipeline configured for AprilTags: the ball detector is 0. */
    private static final int FIDUCIAL_PIPELINE = 1;

    /** Manual velocity step for the dpad up/down buttons. */
    private static final double MANUAL_STEP = ShooterControls.MANUAL_STEP;

    @Override
    public void runOpMode() {
        MecanumDrivebase drive = new MecanumDrivebase(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        ShooterControls shooter = new ShooterControls(new Shooter(hardwareMap));
        LimelightHiveTracker tracker =
                new LimelightHiveTracker(hardwareMap, "limelight", FIDUCIAL_PIPELINE);

        // Same startup as TankTeleOp: the launcher spins up in MANUAL with no button,
        // so the driver only tunes it and feeds. X and Y still allow changing it.
        shooter.setManual(true);
        shooter.setArmed(true);

        boolean fieldCentric = true;
        boolean backWasPressed = false;
        boolean xPrev = false, yPrev = false;
        boolean upPrev = false, downPrev = false;

        telemetry.addLine("Ready. Point the robot downfield before START.");
        telemetry.addData("limelight", tracker.hasLimelight() ? "OK" : "MISSING (shooter = MANUAL)");
        telemetry.addData("launcher", shooter.shooter().hasLauncher() ? "OK" : "MISSING");
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

            double intakePower = 0.0;
            if (gamepad1.right_bumper) intakePower = 1.0;
            else if (gamepad1.left_bumper) intakePower = -1.0;
            intake.setPower(intakePower);

            if (gamepad1.x && !xPrev) shooter.toggleArm();
            if (gamepad1.y && !yPrev) shooter.toggleManual();
            xPrev = gamepad1.x; yPrev = gamepad1.y;

            if (shooter.isManual()) {
                if (gamepad1.dpad_up   && !upPrev)   shooter.nudgeManualVelocity(+MANUAL_STEP);
                if (gamepad1.dpad_down && !downPrev) shooter.nudgeManualVelocity(-MANUAL_STEP);
            }
            upPrev = gamepad1.dpad_up; downPrev = gamepad1.dpad_down;

            Double rangeIn = (shooter.isArmed() && !shooter.isManual())
                    ? tracker.nearestRangeIn() : null;
            shooter.update(rangeIn);
            shooter.requestFeed(gamepad1.a, gamepad1.b);

            telemetry.addData("mode", fieldCentric ? "FIELD-centric" : "ROBOT-centric");
            telemetry.addData("heading", "%.1f deg", Math.toDegrees(drive.getHeading()));
            telemetry.addData("stick", "fwd %+.2f  str %+.2f  turn %+.2f",
                    forward, strafe, turn);
            telemetry.addData("intake", "%+.2f", intakePower);
            telemetry.addData("launcher", "%s  asked %.0f  actual %.0f ticks/s  %s",
                    shooter.velocitySource(), shooter.getCommandedVelocity(),
                    shooter.getVelocity(), shooter.isSpunUp() ? "READY" : "spinning up");
            if (!Double.isNaN(shooter.getLastRangeIn())) {
                telemetry.addData("  range", "%.1f in", shooter.getLastRangeIn());
            }
            telemetry.addData("feed", shooter.feedState());
            telemetry.update();
        }

        drive.stop();
        intake.stop();
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
