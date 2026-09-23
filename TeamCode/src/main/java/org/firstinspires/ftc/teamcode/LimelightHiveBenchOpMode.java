package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
import java.util.Locale;

/**
 * Bench bring-up for reading the BIOBUZZ HIVE off the Limelight 3A.
 *
 * The Limelight sibling of {@link HiveBenchOpMode}. It does the same three jobs --
 * recognition, calibrate the tilt sign/pitch, prove the settled gate -- but against
 * the Limelight's fiducial pipeline rather than a USB webcam, so the signs and the
 * camera height are this camera's own and have to be checked separately.
 *
 * Before running, put the Limelight on a fiducial (AprilTag) pipeline. On the
 * camera's web page: set a pipeline's type to "Fiducial", tag family 36h11, and
 * turn on 3D. The 3D solve is what fills in the camera-space pose this needs for
 * tilt and height; without it you still get IDs and angles, but every observation
 * will read "no 3D pose" and be rejected.
 *
 * Controls:
 *   A  capture the current tilt as the LOW endpoint
 *   B  capture the current tilt as the HIGH endpoint
 *   X  reset the observed tilt range
 *   Y  reset the TIP counter
 */
@TeleOp(name = "4. HIVE Bench (Limelight)", group = "Bringup")
public class LimelightHiveBenchOpMode extends LinearOpMode {

    /** Must match the Robot Configuration name of the Limelight. */
    private static final String LIMELIGHT_NAME = "limelight";

    /**
     * The fiducial pipeline you configured on the camera (0..9). This team runs
     * the ball detector on pipeline 0 and HIVE AprilTags on pipeline 1, so the
     * ball detector is never disturbed by HIVE work.
     */
    private static final int FIDUCIAL_PIPELINE = 1;

    private double tiltMin = Double.NaN, tiltMax = Double.NaN;
    private Double capturedLow = null, capturedHigh = null;

    private HiveGeometry.Slot lastSettledSlot = HiveGeometry.Slot.UNKNOWN;
    private int tipCount = 0;
    private int rejectedFrames = 0, settledFrames = 0;

    private boolean aPrev, bPrev, xPrev, yPrev;

    @Override
    public void runOpMode() {
        LimelightHiveTracker tracker =
                new LimelightHiveTracker(hardwareMap, LIMELIGHT_NAME, FIDUCIAL_PIPELINE);

        telemetry.addLine("Point the Limelight at a HIVE CELL cluster, then press START.");
        telemetry.update();
        waitForStart();

        tracker.start();

        while (opModeIsActive()) {
            LLStatus status = tracker.getLimelight().getStatus();
            telemetry.addData("pipeline", "%d %s", status.getPipelineIndex(), status.getPipelineType());
            telemetry.addData("fps", "%.0f", status.getFps());

            // --- raw recognition: everything seen, HIVE tag or not ---
            List<LimelightHiveTracker.Tag> tags = tracker.recognize();
            if (tags.isEmpty()) {
                telemetry.addLine("\nNo AprilTags in view.");
            }
            for (LimelightHiveTracker.Tag t : tags) {
                telemetry.addLine(LimelightHiveTracker.describe(t));
            }

            // --- the gate ---
            List<LimelightHiveTracker.Observation> obs = tracker.observe();
            for (LimelightHiveTracker.Observation o : obs) {
                trackTilt(o.tiltDeg);
                countTips(o);

                telemetry.addLine(String.format(Locale.US,
                        "\n%s  [%s]  %d/%d tags",
                        o.cell.clusterName, o.cell.name(),
                        o.membersFound, HiveGeometry.CLUSTER_MEMBER_COUNT));
                telemetry.addLine(String.format(Locale.US,
                        "  range %6.1f in   bearing %+6.1f deg   elev %+6.1f deg",
                        o.rangeIn, o.bearingDeg, o.elevationDeg));
                telemetry.addLine(String.format(Locale.US,
                        "  tilt  %+6.1f deg  height  %5.1f in   slot %s",
                        o.tiltDeg, o.heightIn, o.slot));
                telemetry.addLine(o.settled
                        ? "  SETTLED -- safe to localize"
                        : "  REJECTED -- " + o.rejectReason);
            }

            // --- calibration capture ---
            if (gamepad1.a && !aPrev && !obs.isEmpty()) capturedLow  = obs.get(0).tiltDeg;
            if (gamepad1.b && !bPrev && !obs.isEmpty()) capturedHigh = obs.get(0).tiltDeg;
            if (gamepad1.x && !xPrev) { tiltMin = Double.NaN; tiltMax = Double.NaN; }
            if (gamepad1.y && !yPrev) { tipCount = 0; settledFrames = 0; rejectedFrames = 0; }
            aPrev = gamepad1.a; bPrev = gamepad1.b; xPrev = gamepad1.x; yPrev = gamepad1.y;

            telemetry.addLine(String.format(Locale.US,
                    "\ntilt seen: %s .. %s   (expect a %.0f deg spread end to end)",
                    fmt(tiltMin), fmt(tiltMax), 2 * HiveGeometry.TILT_DEG));
            telemetry.addData("frames", "settled %d / rejected %d", settledFrames, rejectedFrames);
            telemetry.addData("TIPs counted", tipCount);

            if (capturedLow != null && capturedHigh != null) {
                telemetry.addLine("\n--- paste into HiveGeometry ---");
                telemetry.addLine(String.format(Locale.US,
                        "SETTLED_TILT_LOW_DEG  = %+.1f;", capturedLow));
                telemetry.addLine(String.format(Locale.US,
                        "SETTLED_TILT_HIGH_DEG = %+.1f;", capturedHigh));
                // The captured endpoints should straddle zero and span ~2*TILT. If
                // they do not, LIMELIGHT_PITCH_DEG is off by the midpoint, and every
                // tilt reading is biased by the same amount -- which reads as
                // "always mid-swing", never as a calibration error. A negative span
                // (high endpoint below low) means LIMELIGHT_TILT_SIGN is backwards.
                double span = capturedHigh - capturedLow;
                double mid  = (capturedHigh + capturedLow) / 2.0;
                telemetry.addLine(String.format(Locale.US,
                        "span %+.1f deg (expect %.0f), midpoint %+.1f deg (expect 0)",
                        span, 2 * HiveGeometry.TILT_DEG, mid));
                if (Math.abs(mid) > HiveGeometry.TILT_TOL_DEG) {
                    telemetry.addLine(String.format(Locale.US,
                            "  -> midpoint is off zero: LIMELIGHT_PITCH_DEG is wrong by ~%+.1f deg", -mid));
                }
                if (span < 0) {
                    telemetry.addLine("  -> high endpoint below low: flip LIMELIGHT_TILT_SIGN");
                } else if (span < HiveGeometry.TILT_DEG) {
                    telemetry.addLine("  -> span too small: did you capture both endpoints?");
                }
            } else {
                telemetry.addLine("\nA = capture LOW endpoint, B = capture HIGH endpoint");
            }

            telemetry.update();
        }

        tracker.close();
    }

    private void trackTilt(double tilt) {
        if (Double.isNaN(tilt)) return;
        if (Double.isNaN(tiltMin) || tilt < tiltMin) tiltMin = tilt;
        if (Double.isNaN(tiltMax) || tilt > tiltMax) tiltMax = tilt;
    }

    /**
     * A TIP is a settled-to-settled transition between opposite slots. Counting it
     * this way rather than by watching for motion means a swing that is observed
     * only at its endpoints still counts, and a wobble that never reaches the other
     * endpoint does not.
     */
    private void countTips(LimelightHiveTracker.Observation o) {
        if (!o.settled) { rejectedFrames++; return; }
        settledFrames++;
        if (lastSettledSlot != HiveGeometry.Slot.UNKNOWN && o.slot != lastSettledSlot) tipCount++;
        lastSettledSlot = o.slot;
    }

    private static String fmt(double v) {
        return Double.isNaN(v) ? "--" : String.format(Locale.US, "%+.1f", v);
    }
}
