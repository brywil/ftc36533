package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Measures the flywheel table used by {@link Shooter}: "at this distance, this much
 * power lands the ball where I want".
 *
 * HOW TO USE IT
 *
 *  1. Put an AprilTag on the thing you are shooting at. It does not have to be a
 *     HIVE tag -- any tag the Limelight's fiducial pipeline can see works, because
 *     all this needs is the distance number.
 *  2. Stand at a distance you care about (your closest shot, say). The range is
 *     read live from the tag, so you do not need a tape measure.
 *  3. Spin the flywheel with the dpad, feed a ball with the right bumper, and tune
 *     the power until shots land where you want.
 *  4. Press A to record {current range, current power} as a point.
 *  5. Repeat at other distances -- three or four points from nearest to farthest.
 *  6. The table prints on the telemetry. Copy it into Shooter.RANGE_POWER_TABLE.
 *
 * Controls:
 *   dpad up/down     flywheel power +/- 0.01
 *   dpad left/right  flywheel power +/- 0.05
 *   right bumper     run the feed motor (hold; this actually launches a ball)
 *   A                record the current range and power as a point
 *   B                clear all recorded points
 *   X                spin the flywheel up / down at the current power
 */
@TeleOp(name = "5. Shooter Calibrate", group = "Bringup")
public class ShooterCalibrateOpMode extends LinearOpMode {

    /** Must match the Robot Configuration name of the Limelight. */
    private static final String LIMELIGHT_NAME = "limelight";

    /** The fiducial pipeline you configured on the camera (0..9). */
    private static final int FIDUCIAL_PIPELINE = 1;

    private static final double POWER_STEP_FINE = 0.01;
    private static final double POWER_STEP_COARSE = 0.05;

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap);
        LimelightHiveTracker tracker =
                new LimelightHiveTracker(hardwareMap, LIMELIGHT_NAME, FIDUCIAL_PIPELINE);

        // Recorded points, as {inches, power}. Sorted only when printed.
        List<double[]> points = new ArrayList<>();

        double power = 0.6;
        boolean spinning = false;

        telemetry.addLine("Point the Limelight at an AprilTag on your target, then START.");
        telemetry.addLine(shooter.hasFlywheel()
                ? "flywheel: OK" : "flywheel: MISSING -- name it \"flywheel\"");
        telemetry.addLine(shooter.hasFeed() ? "feed: OK" : "feed: MISSING (optional)");
        telemetry.update();

        waitForStart();
        tracker.start();

        boolean aPrev = false, bPrev = false, xPrev = false;
        boolean upPrev = false, downPrev = false, leftPrev = false, rightPrev = false;

        while (opModeIsActive()) {
            // --- distance to any tag in view; nearest wins ---
            Double rangeIn = tracker.nearestRangeIn();

            // --- power tuning ---
            if (gamepad1.dpad_up    && !upPrev)    power = clamp(power + POWER_STEP_FINE);
            if (gamepad1.dpad_down  && !downPrev)  power = clamp(power - POWER_STEP_FINE);
            if (gamepad1.dpad_right && !rightPrev) power = clamp(power + POWER_STEP_COARSE);
            if (gamepad1.dpad_left  && !leftPrev)  power = clamp(power - POWER_STEP_COARSE);
            upPrev = gamepad1.dpad_up; downPrev = gamepad1.dpad_down;
            leftPrev = gamepad1.dpad_left; rightPrev = gamepad1.dpad_right;

            if (gamepad1.x && !xPrev) spinning = !spinning;
            xPrev = gamepad1.x;
            shooter.setPower(spinning ? power : 0.0);

            // --- feed: hold to launch; letting go stops, which is the safe failure ---
            shooter.setFeed(gamepad1.right_bumper ? 1.0 : 0.0);

            // --- record ---
            if (gamepad1.a && !aPrev) {
                if (rangeIn == null) {
                    telemetry.addLine("Cannot record: no tag in view for a range.");
                } else {
                    points.add(new double[] { rangeIn, power });
                }
            }
            if (gamepad1.b && !bPrev) points.clear();
            aPrev = gamepad1.a; bPrev = gamepad1.b;

            // --- telemetry ---
            telemetry.addData("range to tag", rangeIn == null
                    ? "no tag in view" : String.format(Locale.US, "%.1f in", rangeIn));
            telemetry.addData("flywheel power", "%.2f %s", power, spinning ? "(spinning)" : "(stopped)");
            telemetry.addData("feed", gamepad1.right_bumper ? "RUNNING" : "off");
            telemetry.addLine();
            telemetry.addData("points recorded", points.size());

            // Print the table sorted nearest-first, ready to paste.
            List<double[]> sorted = new ArrayList<>(points);
            sorted.sort((p, q) -> Double.compare(p[0], q[0]));
            telemetry.addLine("--- paste into Shooter.RANGE_POWER_TABLE ---");
            telemetry.addLine("public static final double[][] RANGE_POWER_TABLE = {");
            for (double[] p : sorted) {
                telemetry.addLine(String.format(Locale.US, "    { %.1f, %.2f },", p[0], p[1]));
            }
            telemetry.addLine("};");
            telemetry.addLine();
            telemetry.addLine("A = record point   B = clear   X = spin on/off");
            telemetry.update();
        }

        shooter.stop();
        tracker.close();
    }

    private static double clamp(double v) {
        if (v > 1.0) return 1.0;
        if (v < 0.0) return 0.0;
        return v;
    }
}
