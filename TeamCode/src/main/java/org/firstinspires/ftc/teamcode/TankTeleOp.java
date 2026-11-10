package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

/**
 * Tank TeleOp for the goBILDA BIOBUZZ StarterBot / kitbot -- one motor per side.
 *
 * The counterpart to {@link MecanumTeleOp}. It drives the same way by default (left
 * stick translate, right stick turn) so a driver can move between the practice bot and
 * the kitbot without relearning anything, but there is no strafe and no field-centric
 * mode: a skid-steer cannot slide sideways.
 *
 *   left stick      translate (forward / back)
 *   right stick x   turn
 *   right trigger   precision creep -- scales everything down for lining up
 *   back            toggle ARCADE steering / TANK steering
 *   right bumper    intake in (hold);  left bumper  intake out -- clears a jam
 *   X               arm / disarm the launcher
 *   Y               AUTO (from the tag distance) / MANUAL (fixed velocity)
 *   A               feed a ball (hold) -- only once the wheel is up to speed
 *   B               reverse the feed -- clears a jam
 *   dpad up/down    manual velocity +/- (when in MANUAL)
 *
 * Every button here is distinct; nothing doubles up. The right trigger stays
 * precision creep and is NOT the intake, so the intake is on the bumpers instead.
 *
 * The intake is the motor + two side servos, driven together; the launcher is the
 * encoder motor; the feed is the windmill servo. All names match goBILDA's own
 * BIOBUZZ example so one Robot Configuration drives both. See {@link Intake} and
 * {@link Shooter}.
 *
 * ARCADE steering is the default; TANK steering drives each side from its own stick.
 */
@Disabled
@TeleOp(name = "2. Tank TeleOp (kitbot)", group = "Drive")
public class TankTeleOp extends LinearOpMode {

    /** Gamepad sticks rarely read exactly zero when you let go. Below this is "not touched". */
    private static final double DEADBAND = 0.05;

    /** Multiplier at full precision trigger. */
    private static final double CREEP_SCALE = 0.30;

    /**
     * Top speed, as a fraction of full power. Turn this down to slow the robot down
     * for testing -- 1.0 is full speed. This is the knob for "it drives too fast",
     * and it applies to both ARCADE and TANK steering. The precision trigger still
     * works on top of it (it scales further down).
     */
    private static final double DRIVE_GAIN = 1.0;

    /** The Limelight pipeline configured for AprilTags: the ball detector is 0. */
    private static final int FIDUCIAL_PIPELINE = 1;

    /** Manual velocity step for the dpad up/down buttons. */
    private static final double MANUAL_STEP = ShooterControls.MANUAL_STEP;

    @Override
    public void runOpMode() {
        TankDrivebase drive = new TankDrivebase(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        ShooterControls shooter = new ShooterControls(new Shooter(hardwareMap));
        // Half-speed feed in TeleOp only; autonomous keeps full power.
        shooter.setForwardFeedPower(ShooterControls.TELEOP_FEED_POWER);
        LimelightHiveTracker tracker =
                new LimelightHiveTracker(hardwareMap, "limelight", FIDUCIAL_PIPELINE);
        // No camera means AUTO can never answer; fall back to MANUAL up front.
        if (!tracker.hasLimelight()) shooter.fallBackToManual();

        boolean tankSteering = false;   
        boolean backWasPressed = false;
        boolean xPrev = false, yPrev = false;
        boolean upPrev = false, downPrev = false;

        telemetry.addLine("Ready. Point the robot downfield before START.");
        telemetry.addData("limelight", tracker.hasLimelight() ? "OK" : "MISSING (shooter = MANUAL)");
        telemetry.addData("launcher", shooter.shooter().hasLauncher() ? "OK" : "MISSING");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
        tracker.start();

        while (opModeIsActive()) {
            // Gamepad y is negative when pushed forward; flip it so +forward is forward.
            double forward = deadband(-gamepad1.left_stick_y);
            double turn    = deadband(gamepad1.right_stick_x);

            // Squaring the stick makes small pushes gentler while a full push still
            // gives full power -- fine control near center.
            forward = square(forward);
            turn    = square(turn);

            // DRIVE_GAIN caps everything; the precision trigger scales down further.
            double scale = DRIVE_GAIN * (1.0 - (1.0 - CREEP_SCALE) * gamepad1.right_trigger);
            forward *= scale;
            turn    *= scale;

            if (gamepad1.back && !backWasPressed) {
                tankSteering = !tankSteering;
            }
            backWasPressed = gamepad1.back;

            if (tankSteering) {
                double leftSide  = forward;
                // The right stick is squared but not yet scaled, so apply the same
                // scale it would get in arcade -- otherwise TANK steering bypasses
                // DRIVE_GAIN and the creep trigger.
                double rightSide = square(deadband(-gamepad1.right_stick_y)) * scale;
                drive.driveTank(leftSide, rightSide);
            } else {
                drive.driveRobotCentric(forward, turn);
            }

            // Intake: hold right bumper to pull in, left bumper to push out (clears a
            // jam). Hold-to-run, so letting go is always the safe stop.
            double intakePower = 0.0;
            if (gamepad1.right_bumper) intakePower = 1.0;
            else if (gamepad1.left_bumper) intakePower = -1.0;
            intake.setPower(intakePower);

            // Launcher on the driver's spare buttons.
            if (gamepad1.x && !xPrev) shooter.toggleArm();
            if (gamepad1.y && !yPrev) shooter.toggleManual();
            xPrev = gamepad1.x; yPrev = gamepad1.y;

            if (shooter.isManual()) {
                if (gamepad1.dpad_up   && !upPrev)   shooter.nudgeManualVelocity(+MANUAL_STEP);
                if (gamepad1.dpad_down && !downPrev) shooter.nudgeManualVelocity(-MANUAL_STEP);
            }
            upPrev = gamepad1.dpad_up; downPrev = gamepad1.dpad_down;

            // Only ask the camera when it can matter: armed and in AUTO.
            Double rangeIn = (shooter.isArmed() && !shooter.isManual())
                    ? tracker.nearestRangeIn() : null;
            shooter.update(rangeIn);
            shooter.requestFeed(gamepad1.a, gamepad1.b);

            telemetry.addData("steering", tankSteering ? "TANK" : "ARCADE");
            telemetry.addData("stick", "fwd %+.2f  turn %+.2f", forward, turn);
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
