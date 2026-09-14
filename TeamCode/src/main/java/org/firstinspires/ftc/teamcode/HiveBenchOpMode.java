package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;
import java.util.Locale;

/**
 * Bench bring-up and proof of concept for HIVE localization.
 *
 * Built for a PARTIAL field -- one lifted CELL with one cluster of four AprilTags
 * is enough to run every check in here. It does not need a pivot, a second CELL,
 * or any field coordinates, because nothing it measures is field-referenced: tilt
 * is against gravity and height is against the TILES.
 *
 * What it is for, in order:
 *
 *  1. CALIBRATE. The camera pitch and the sign of ftcPose.pitch are the two
 *     constants you cannot derive, only observe. Hold the CELL at each endpoint,
 *     press A and B, and it prints the constants to paste into HiveGeometry.
 *  2. PROVE THE GATE. Swing the CELL by hand through its arc and watch SETTLED
 *     drop out in the middle and come back at the ends. That is the entire
 *     mid-swing rejection story, testable on a bench.
 *  3. MAP THE ENVELOPE. Walk the camera back with a tape measure and read where
 *     the cluster stops solving at 100%. That number decides where the camera
 *     gets mounted, and it is cheaper to learn now than on a field.
 *
 * Controls:
 *   A  capture the current tilt as the LOW endpoint
 *   B  capture the current tilt as the HIGH endpoint
 *   X  reset the observed tilt range
 *   Y  reset the TIP counter
 */
@TeleOp(name = "3. HIVE Bench (AprilTag)", group = "Bringup")
public class HiveBenchOpMode extends LinearOpMode {

    /** Must match the Robot Configuration name of your USB camera. */
    private static final String WEBCAM_NAME = "Webcam 1";

    private double tiltMin = Double.NaN, tiltMax = Double.NaN;
    private Double capturedLow = null, capturedHigh = null;

    private HiveGeometry.Slot lastSettledSlot = HiveGeometry.Slot.UNKNOWN;
    private int tipCount = 0;
    private int rejectedFrames = 0, settledFrames = 0;

    private boolean aPrev, bPrev, xPrev, yPrev;

    @Override
    public void runOpMode() {
        HiveTracker tracker = new HiveTracker(hardwareMap, WEBCAM_NAME);

        telemetry.addLine("Point the camera at a HIVE CELL cluster, then press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            List<HiveTracker.Observation> obs = tracker.observe();

            telemetry.addData("camera", tracker.getPortal().getCameraState());
            telemetry.addData("fps", "%.1f", tracker.getPortal().getFps());

            if (obs.isEmpty()) {
                telemetry.addLine("\nNo HIVE cluster in view.");
                telemetry.addLine("If you see tags in the preview but nothing here, the");
                telemetry.addLine("cluster is only partly visible -- all four members are");
                telemetry.addLine("needed before the SDK reports a cluster detection.");
            }

            for (HiveTracker.Observation o : obs) {
                trackTilt(o.tiltDeg);
                countTips(o);

                telemetry.addLine(String.format(Locale.US,
                        "\n%s  [%s]  %d%%", o.cell.clusterName, o.cell.name(), o.percentClusterFound));
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
                // The captured endpoints should straddle zero and span ~2*TILT. If they
                // do not, the camera pitch constant is off, and every tilt reading is
                // biased by the same amount -- which reads as "always mid-swing".
                double span = Math.abs(capturedHigh - capturedLow);
                double mid  = (capturedHigh + capturedLow) / 2.0;
                telemetry.addLine(String.format(Locale.US,
                        "span %.1f deg (expect %.0f), midpoint %+.1f deg (expect 0)",
                        span, 2 * HiveGeometry.TILT_DEG, mid));
                if (Math.abs(mid) > HiveGeometry.TILT_TOL_DEG) {
                    telemetry.addLine(String.format(Locale.US,
                            "  -> midpoint is off zero: CAMERA_PITCH_DEG is wrong by ~%+.1f deg", -mid));
                }
                if (span < HiveGeometry.TILT_DEG) {
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
        if (Double.isNaN(tiltMin) || tilt < tiltMin) tiltMin = tilt;
        if (Double.isNaN(tiltMax) || tilt > tiltMax) tiltMax = tilt;
    }

    /**
     * A TIP is a settled-to-settled transition between opposite slots. Counting it
     * this way rather than by watching for motion means a swing that is observed
     * only at its endpoints still counts, and a wobble that never reaches the other
     * endpoint does not.
     */
    private void countTips(HiveTracker.Observation o) {
        if (!o.settled) { rejectedFrames++; return; }
        settledFrames++;
        if (lastSettledSlot != HiveGeometry.Slot.UNKNOWN && o.slot != lastSettledSlot) tipCount++;
        lastSettledSlot = o.slot;
    }

    private static String fmt(double v) {
        return Double.isNaN(v) ? "--" : String.format(Locale.US, "%+.1f", v);
    }
}
