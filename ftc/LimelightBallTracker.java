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
 *   [0] source      0 none, 1 contour path, 2 Hough fallback
 *   [1] tx deg      [2] ty deg      [3] distance m
 *   [4] cx px       [5] cy px       [6] radius px
 *   [7] confidence  0..1 -- circularity on the contour path, disk fill on the
 *                   Hough path. Different measures, hence different gates.
 *
 * Source 2 means the ball was fused into a same-hue blob -- a bumper, a wall,
 * another ball -- and was recovered by a Hough pass from inside it. A real
 * detection, but the degraded path: gated looser, and its range is worth less.
 */
public class LimelightBallTracker {

    public static final int SOURCE_NONE    = 0;
    public static final int SOURCE_CONTOUR = 1;
    public static final int SOURCE_HOUGH   = 2;

    /** Reject a result older than this; a stale frame aims at where the ball WAS. */
    private static final long MAX_STALENESS_MS = 200;

    /**
     * Tighter than the pipeline's own MIN_CIRC of 0.65. The pipeline decides what to
     * draw on the stream; the robot decides what to drive at.
     */
    private static final double MIN_CONF_CONTOUR = 0.75;
    private static final double MIN_CONF_HOUGH   = 0.55;

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
        public final double tx, ty, distanceMeters, radiusPx, confidence;
        public final int source;

        Target(double tx, double ty, double distanceMeters,
               double radiusPx, double confidence, int source) {
            this.tx = tx;
            this.ty = ty;
            this.distanceMeters = distanceMeters;
            this.radiusPx = radiusPx;
            this.confidence = confidence;
            this.source = source;
        }

        /** True when this came out of a same-hue blob; range is less trustworthy. */
        public boolean isFused() { return source == SOURCE_HOUGH; }
    }

    /** @return the current target, or null when there is nothing worth driving at. */
    public Target getTarget() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;
        if (result.getStaleness() > MAX_STALENESS_MS) return null;

        double[] py = result.getPythonOutput();
        if (py == null || py.length < 8) return null;

        int source = (int) Math.round(py[0]);
        if (source == SOURCE_NONE) return null;

        double minConf;
        if (source == SOURCE_CONTOUR) {
            minConf = MIN_CONF_CONTOUR;
        } else if (source == SOURCE_HOUGH) {
            minConf = MIN_CONF_HOUGH;
        } else {
            return null;   // pipeline newer than this file; do not guess
        }
        if (py[7] < minConf) return null;

        return new Target(py[1], py[2], py[3], py[6], py[7], source);
    }
}
