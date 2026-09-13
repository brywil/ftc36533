package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * FTC-side read of the SnapScript in snapscript/yellow_waffle_ball.py.
 *
 * FTC does NOT use NetworkTables -- that is the FRC path, and it is what
 * robot/read_llpython.py in this repo speaks. On FTC the Limelight is a hardware
 * device in the Robot Configuration (add it as "Ethernet Device" -> Limelight3A,
 * named "limelight"), and the pipeline's llpython array arrives as
 * LLResult.getPythonOutput().
 *
 * The array layout, set by the pipeline:
 *
 *   [0] valid flag   [1] tx deg   [2] ty deg      [3] distance m
 *   [4] cx px        [5] cy px    [6] radius px   [7] circularity 0..1
 */
public class LimelightBallTracker {

    /** Reject a result older than this; a stale frame aims at where the ball WAS. */
    private static final long MAX_STALENESS_MS = 200;

    /**
     * Tighter than the pipeline's own MIN_CIRC of 0.65. The pipeline decides what to
     * draw on the stream; the robot decides what to drive at.
     */
    private static final double MIN_CIRCULARITY = 0.75;

    private final Limelight3A limelight;

    public LimelightBallTracker(HardwareMap hardwareMap, int pipelineIndex) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipelineIndex);
        limelight.setPollRateHz(100);
    }

    public void start() { limelight.start(); }
    public void stop()  { limelight.stop();  }

    /** One detected ball. Angles in degrees, range in meters. */
    public static class Target {
        public final double tx, ty, distanceMeters, radiusPx, circularity;

        Target(double tx, double ty, double distanceMeters,
               double radiusPx, double circularity) {
            this.tx = tx;
            this.ty = ty;
            this.distanceMeters = distanceMeters;
            this.radiusPx = radiusPx;
            this.circularity = circularity;
        }
    }

    /** @return the current target, or null when there is nothing worth driving at. */
    public Target getTarget() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;
        if (result.getStaleness() > MAX_STALENESS_MS) return null;

        double[] py = result.getPythonOutput();
        if (py == null || py.length < 8) return null;
        if (py[0] < 0.5) return null;
        if (py[7] < MIN_CIRCULARITY) return null;

        return new Target(py[1], py[2], py[3], py[6], py[7]);
    }
}
