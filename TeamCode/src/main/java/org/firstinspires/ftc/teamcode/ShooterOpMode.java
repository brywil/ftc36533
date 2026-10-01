package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

/**
 * Shoots at whatever AprilTag the Limelight is looking at, choosing launcher speed
 * from the distance to that tag via {@link Shooter}'s measured table.
 *
 * Nothing here aims the robot -- turn it so the tag is in frame yourself. This OpMode
 * only answers "how fast should the wheel spin from here", and holds the feed back
 * until the wheel is actually up to speed (a real measurement, thanks to the encoder).
 *
 * The arm / manual / spin-up / feed logic lives in {@link ShooterControls}, shared with
 * the drive TeleOps, so this file is only the controls and the telemetry.
 *
 * Controls:
 *   A              toggle the launcher master switch on/off
 *   B              toggle MANUAL (fixed speed) / AUTO (from the tag distance)
 *   dpad up/down   manual speed, when in MANUAL
 *   right bumper   feed a ball (hold). Only works once the wheel is up to speed.
 *   left bumper    reverse the feed -- clears a jam
 *
 * Run "5. Shooter Calibrate" first and paste the points into
 * Shooter.RANGE_VELOCITY_TABLE. Until at least Shooter.MIN_POINTS are present this
 * refuses to auto-fire, and says so.
 */
@Disabled
@TeleOp(name = "6. Shooter (Limelight)", group = "Drive")
public class ShooterOpMode extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "limelight";
    private static final int FIDUCIAL_PIPELINE = 1;

    private static final double MANUAL_STEP = ShooterControls.MANUAL_STEP;

    @Override
    public void runOpMode() {
        ShooterControls shooter = new ShooterControls(new Shooter(hardwareMap));
        LimelightHiveTracker tracker =
                new LimelightHiveTracker(hardwareMap, LIMELIGHT_NAME, FIDUCIAL_PIPELINE);
        if (!tracker.hasLimelight()) shooter.fallBackToManual();

        telemetry.addLine("Point the Limelight at the target's AprilTag, then START.");
        telemetry.addData("limelight", tracker.hasLimelight() ? "OK" : "MISSING (MANUAL only)");
        telemetry.addData("launcher", shooter.shooter().hasLauncher() ? "OK" : "MISSING");
        telemetry.addData("table points", Shooter.RANGE_VELOCITY_TABLE.length
                + (Shooter.RANGE_VELOCITY_TABLE.length < Shooter.MIN_POINTS
                   ? "  -- TOO FEW: calibrate first" : ""));
        telemetry.update();

        waitForStart();
        tracker.start();

        boolean aPrev = false, bPrev = false;
        boolean upPrev = false, downPrev = false;

        while (opModeIsActive()) {
            if (gamepad1.a && !aPrev) shooter.toggleArm();
            if (gamepad1.b && !bPrev) shooter.toggleManual();
            aPrev = gamepad1.a; bPrev = gamepad1.b;

            if (shooter.isManual()) {
                if (gamepad1.dpad_up   && !upPrev)   shooter.nudgeManualVelocity(+MANUAL_STEP);
                if (gamepad1.dpad_down && !downPrev) shooter.nudgeManualVelocity(-MANUAL_STEP);
            }
            upPrev = gamepad1.dpad_up; downPrev = gamepad1.dpad_down;

            // Only ask the camera when it can matter: armed and in AUTO.
            Double rangeIn = (shooter.isArmed() && !shooter.isManual())
                    ? tracker.nearestRangeIn() : null;
            shooter.update(rangeIn);
            shooter.requestFeed(gamepad1.right_bumper, gamepad1.left_bumper);

            // --- telemetry ---
            double range = shooter.getLastRangeIn();
            telemetry.addData("range", Double.isNaN(range)
                    ? "no tag in view" : String.format(Locale.US, "%.1f in", range));
            telemetry.addData("mode", shooter.isManual() ? "MANUAL" : "AUTO");
            telemetry.addData("wheel", "%s  asked %.0f  actual %.0f ticks/s  %s",
                    shooter.velocitySource(), shooter.getCommandedVelocity(),
                    shooter.getVelocity(), shooter.isSpunUp() ? "READY" : "spinning up");
            telemetry.addData("feed", shooter.feedState());
            telemetry.addLine();
            telemetry.addLine("A arm/off   B manual/auto   dpad tune (manual)   RB feed   LB clear");
            telemetry.update();
        }

        shooter.stop();
        tracker.close();
    }
}
