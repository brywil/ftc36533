package org.firstinspires.ftc.teamcode;

/**
 * BIOBUZZ HIVE geometry, and the constants the localizer gates on.
 *
 * Every value in the MANUAL block below is quoted from the BIOBUZZ Competition
 * Manual V1 with its source, so it can be re-checked when a Team Update moves it.
 * Every value in the MEASURE block is a placeholder that must be replaced with a
 * number you took off the real hardware -- they are wrong until you do.
 *
 * Why the HIVE can be localized against at all: it is a seesaw on a fixed pivot
 * with two stable endpoints held by dampers. It never rests in between. So a
 * settled HIVE presents its tag clusters at one of two known poses, and anything
 * else is a swing in progress and must be discarded rather than believed.
 */
public final class HiveGeometry {

    private HiveGeometry() {}

    // ------------------------------------------------------------ MANUAL
    /** Section 9.9: 3.25 in. (8.25 cm) square, 36h11 family. */
    public static final double TAG_SIZE_IN = 3.25;

    /** Figure 9-10: seesaw tilt at each stable endpoint. */
    public static final double TILT_DEG = 30.0;

    /** Section 9.6.1: "The Frame supports 2 pivots with their axis 43.95 in. above the TILES." */
    public static final double PIVOT_HEIGHT_IN = 43.95;

    /** Figure 9-9: each CELL sits this far from the pivot along the connecting assembly. */
    public static final double CELL_OFFSET_IN = 18.84;

    /** Figure 9-10: "Bottom of HIVE above TILES" -- the low CELL's tag plane. */
    public static final double LOW_TAG_HEIGHT_IN = 25.5;

    /** Figure 9-10: the two HIVES sit this far apart, side by side at FIELD center. */
    public static final double HIVE_CENTER_TO_CENTER_IN = 25.5;

    /**
     * The high CELL's tag plane. Derived rather than quoted, and the derivation is
     * its own cross-check: rotating the assembly through 2*TILT lifts a CELL by
     * 2 * CELL_OFFSET * sin(TILT) = 18.84 in., which lands the high plane at 44.3 in.
     * -- consistent with the pivot at 43.95 in. with CELLS hung either side of it.
     * If those two numbers ever stop agreeing, the manual changed and so must this.
     */
    public static final double HIGH_TAG_HEIGHT_IN =
            LOW_TAG_HEIGHT_IN + 2.0 * CELL_OFFSET_IN * Math.sin(Math.toRadians(TILT_DEG));

    /**
     * The four AprilTag Clusters, by the ID of their first member (Section 9.9).
     * Each CELL carries its own cluster on its bottom face, so the ID range tells
     * you WHICH CELL you are looking at -- never which state the HIVE is in. Both
     * CELLS of a HIVE face downward at all times and are visible together.
     */
    public enum Cell {
        RED_FAR      (30, "RED SCORING",   Alliance.RED),
        RED_AUDIENCE (34, "RED AUDIENCE",  Alliance.RED),
        BLUE_AUDIENCE(38, "BLUE AUDIENCE", Alliance.BLUE),
        BLUE_FAR     (42, "BLUE SCORING",  Alliance.BLUE);

        public final int baseId;
        public final String clusterName;
        public final Alliance alliance;

        Cell(int baseId, String clusterName, Alliance alliance) {
            this.baseId = baseId;
            this.clusterName = clusterName;
            this.alliance = alliance;
        }

        /** The CELL at the other end of the same seesaw. */
        public Cell opposite() {
            switch (this) {
                case RED_FAR:       return RED_AUDIENCE;
                case RED_AUDIENCE:  return RED_FAR;
                case BLUE_AUDIENCE: return BLUE_FAR;
                default:            return BLUE_AUDIENCE;
            }
        }

        public static Cell fromTagId(int id) {
            for (Cell c : values()) {
                if (id >= c.baseId && id <= c.baseId + 3) return c;
            }
            return null;
        }
    }

    public enum Alliance { RED, BLUE }

    /** Which end of the seesaw a CELL is currently at. */
    public enum Slot { LOW, HIGH, UNKNOWN }

    // ------------------------------------------------------------ MEASURE
    /*
     * Camera mounting. Both of these are wrong until measured on your robot, and
     * both feed the gates directly -- a bad CAMERA_PITCH_DEG biases every tilt
     * reading by the same amount, which looks like the HIVE being permanently
     * mid-swing rather than like a calibration error.
     */
    public static final double CAMERA_HEIGHT_IN = 8.0;
    public static final double CAMERA_PITCH_DEG = 20.0;   // up from horizontal, positive

    /**
     * Sign of AprilTagPoseFtc.pitch relative to "tag leaning away from the camera".
     * Do not reason this out -- run HiveBenchOpMode, tilt the CELL, and see which
     * way the number moves. Set to -1.0 if raising the far edge makes pitch fall.
     */
    public static final double PITCH_SIGN = 1.0;

    /**
     * Settled tilt of a CELL's tag plane measured from horizontal, for each slot.
     * Nominally +/- TILT_DEG, but the bench OpMode can capture what your actual
     * hardware does; prefer the measured number, since it folds in whatever the
     * damper stop really gives you.
     */
    public static final double SETTLED_TILT_LOW_DEG  = -TILT_DEG;
    public static final double SETTLED_TILT_HIGH_DEG = +TILT_DEG;

    // ------------------------------------------------------------ GATES
    /*
     * Acceptance bands. The endpoints are 2*TILT = 60 deg apart and the slots are
     * 18.8 in. apart, so these can be generous and still leave a wide dead zone in
     * the middle. Generous is what you want: a tight band rejects good fixes at
     * long range where noise grows, and buys nothing, because a mid-swing HIVE is
     * nowhere near either endpoint.
     */
    public static final double TILT_TOL_DEG  = 12.0;
    public static final double HEIGHT_TOL_IN = 6.0;

    /** Require the full cluster. A partial solve on 3.25 in. tags is not worth trusting. */
    public static final int MIN_CLUSTER_PERCENT = 100;

    /**
     * Reject solves seen at a steep incidence to the tag plane. The tags lean 30 deg
     * outward, so this is far less restrictive than it sounds -- at 6 ft of standoff
     * the incidence is only about 46 deg on the low CELL and 33 deg on the high one.
     */
    public static final double MAX_INCIDENCE_DEG = 60.0;

    // ------------------------------------------------------- FIELD POSES
    /**
     * Field poses of the two seesaw SLOTS for each HIVE. NOT YET MEASURED.
     *
     * The factorisation that matters: SLOT poses are static field geometry and
     * never move, while the HIVE state is only a permutation saying which CELL is
     * currently in which slot. That keeps the constants below immutable and puts
     * all the time-varying part in one enum.
     *
     * Section 9.9 says the Reference Holes "can be used to measure the location of
     * the AprilTag Cluster relative to the rest of the FIELD" -- that is the
     * intended procedure, and why the SDK ships every cluster at (0,0,0).
     */
    public static final boolean SLOT_POSES_MEASURED = false;
}
