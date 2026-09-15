package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * FTC-side read of the SnapScript in snapscript/ball_detector.py.
 *
 * Add the camera to the Robot Configuration as an Ethernet Device -> Limelight3A
 * named "limelight". The pipeline's llpython array arrives as
 * LLResult.getPythonOutput().
 *
 * getLatestResult() returns null until start() has been called, so call start()
 * in your OpMode's init and stop() when it ends.
 *
 * The array layout, set by the pipeline:
 *
 *   [0] what and how, packed as class * 10 + source:
 *         0  nothing found
 *        11  POLLEN by contour        12  POLLEN by Hough recovery
 *        21  NECTAR red by contour    22  NECTAR red by Hough
 *        31  NECTAR blue by contour   32  NECTAR blue by Hough
 *       Any non-zero value means something was found, so the simple check is
 *       just "is index 0 not zero".
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

    /** What kind of ball the camera is looking at. */
    public enum Ball {
        POLLEN(1), NECTAR_RED(2), NECTAR_BLUE(3);

        public final int id;
        Ball(int id) { this.id = id; }

        static Ball fromId(int id) {
            for (Ball b : values()) if (b.id == id) return b;
            return null;
        }
    }

    /**
     * Ask the camera to hunt one kind of ball. This is worth doing: searching one
     * colour is about three times cheaper than searching all three, and the
     * Limelight's processor is not fast. Pass null to hunt everything.
     */
    public void setWanted(Ball ball) {
        limelight.updatePythonInputs(new double[] { ball == null ? 0 : ball.id,
                                                    0, 0, 0, 0, 0, 0, 0 });
    }

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
        /** Which of the three balls this is. */
        public final Ball ball;

        Target(Ball ball, double tx, double ty, double distanceMeters,
               double radiusPx, double confidence, int source) {
            this.ball = ball;
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

        int packed = (int) Math.round(py[0]);
        if (packed == SOURCE_NONE) return null;
        Ball ball = Ball.fromId(packed / 10);
        int source = packed % 10;
        if (ball == null) return null;   // pipeline newer than this file; do not guess

        double minConf;
        if (source == SOURCE_CONTOUR) {
            minConf = MIN_CONF_CONTOUR;
        } else if (source == SOURCE_HOUGH) {
            minConf = MIN_CONF_HOUGH;
        } else {
            return null;   // pipeline newer than this file; do not guess
        }
        if (py[7] < minConf) return null;

        return new Target(ball, py[1], py[2], py[3], py[6], py[7], source);
    }
}
