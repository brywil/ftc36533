package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

/**
 * Shoots at whatever AprilTag the Limelight is looking at, choosing flywheel power
 * from the distance to that tag via {@link Shooter}'s measured table.
 *
 * Nothing here aims the robot -- turn it so the tag is in frame yourself. This
 * OpMode only answers "how hard should the wheel spin from here", and holds the
 * feed back until the wheel is up to speed.
 *
 * Controls:
 *   right bumper   feed a ball (hold). Only works once the wheel is up to speed.
 *   left bumper    reverse the feed -- clears a jam
 *   A              toggle the flywheel master switch on/off
 *   B              hold the wheel at a fixed power (MANUAL) / go back to AUTO
 *   dpad up/down   manual power, when in MANUAL
 *
 * The table starts as placeholders, so run "5. Shooter Calibrate" first and paste
 * the measured points into Shooter.RANGE_POWER_TABLE. Until at least
 * Shooter.MIN_POINTS are present this refuses to auto-fire, and says so.
 */
@TeleOp(name = "6. Shooter (Limelight)", group = "Drive")
public class ShooterOpMode extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "limelight";
    private static final int FIDUCIAL_PIPELINE = 1;

    /**
     * How long to wait after the commanded power changes before letting a ball in.
     * There is no encoder, so the wheel's actual speed cannot be measured; this is
     * the open-loop stand-in for "it has spun up". Too short and the first ball
     * leaves slow; too long and shots get sluggish. Tune it by ear first, then by
     * where the first ball of a burst lands.
     */
    private static final long SPINUP_MS = 700;

    /** Power changes smaller than this are drift, not a new shot -- do not re-wait. */
    private static final double POWER_EPSILON = 0.02;

    private static final double MANUAL_STEP = 0.02;

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap);
        LimelightHiveTracker tracker =
                new LimelightHiveTracker(hardwareMap, LIMELIGHT_NAME, FIDUCIAL_PIPELINE);

        boolean masterOn = false;      // flywheel enabled
        boolean manual = false;        // fixed power instead of table lookup
        double manualPower = 0.6;

        double commandedPower = 0.0;
        long powerChangedAtMs = 0L;

        telemetry.addLine("Point the Limelight at the target's AprilTag, then START.");
        telemetry.addData("table points", Shooter.RANGE_POWER_TABLE.length
                + (Shooter.RANGE_POWER_TABLE.length < Shooter.MIN_POINTS
                   ? "  -- TOO FEW: calibrate first" : ""));
        telemetry.update();

        waitForStart();
        tracker.start();

        boolean aPrev = false, bPrev = false;
        boolean upPrev = false, downPrev = false;

        while (opModeIsActive()) {
            Double rangeIn = tracker.nearestRangeIn();
            Double tablePower = (rangeIn == null) ? null : shooter.rangeToPower(rangeIn);

            if (gamepad1.a && !aPrev) masterOn = !masterOn;
            if (gamepad1.b && !bPrev) manual = !manual;
            aPrev = gamepad1.a; bPrev = gamepad1.b;

            if (manual) {
                if (gamepad1.dpad_up   && !upPrev) manualPower = clamp(manualPower + MANUAL_STEP);
                if (gamepad1.dpad_down && !downPrev) manualPower = clamp(manualPower - MANUAL_STEP);
            }
            upPrev = gamepad1.dpad_up; downPrev = gamepad1.dpad_down;

            // Decide the target power for this instant.
            double targetPower;
            if (!masterOn) {
                targetPower = 0.0;
            } else if (manual) {
                targetPower = manualPower;
            } else if (tablePower != null) {
                targetPower = tablePower;
            } else {
                targetPower = 0.0;   // no tag, or table too small -- do not guess
            }

            if (Math.abs(targetPower - commandedPower) > POWER_EPSILON) {
                commandedPower = targetPower;
                powerChangedAtMs = System.currentTimeMillis();
            }
            shooter.setPower(commandedPower);

            // Spin-up guard: after any power change the wheel is not at speed yet.
            boolean spunUp = commandedPower > 0.0
                    && (System.currentTimeMillis() - powerChangedAtMs) >= SPINUP_MS;

            // Feed. Hold-to-run; reverse clears a jam. Never feed a cold wheel, and
            // never feed when the table could not answer (commandedPower == 0).
            double feedPower = 0.0;
            if (gamepad1.right_bumper && spunUp) feedPower = 1.0;
            else if (gamepad1.left_bumper) feedPower = -1.0;
            shooter.setFeed(feedPower);

            // --- telemetry ---
            telemetry.addData("range", rangeIn == null
                    ? "no tag in view" : String.format(Locale.US, "%.1f in", rangeIn));
            if (manual) {
                telemetry.addData("power (MANUAL)", "%.2f", manualPower);
            } else if (tablePower == null) {
                telemetry.addData("power (AUTO)", "table needs >= %d points -- see 5. Shooter Calibrate",
                        Shooter.MIN_POINTS);
            } else {
                telemetry.addData("power (AUTO)", "%.2f  (from table)", tablePower);
            }
            telemetry.addData("flywheel", masterOn
                    ? String.format(Locale.US, "%.2f %s", commandedPower, spunUp ? "READY" : "spinning up")
                    : "OFF (A to arm)");
            telemetry.addData("feed", feedPower > 0 ? "RUNNING"
                    : feedPower < 0 ? "REVERSE" : (spunUp ? "ready (RB)" : "held"));
            telemetry.addLine();
            telemetry.addLine("A arm/off   B manual/auto   dpad tune (manual)   RB feed   LB clear");
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
