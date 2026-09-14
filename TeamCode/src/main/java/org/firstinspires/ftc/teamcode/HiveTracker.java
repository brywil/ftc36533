package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * Reads the BIOBUZZ HIVE AprilTag clusters and decides whether what it is looking
 * at is trustworthy enough to localize against.
 *
 * The problem this exists to solve: the HIVE is a seesaw that only rests at two
 * endpoints, but it passes through every angle in between on a TIP. A pose solve
 * taken mid-swing does not fail -- it silently returns a confident answer against
 * a cluster that is not where the map says it is. So every observation is gated
 * on two independent physical quantities before it is called settled:
 *
 *   tilt   the tag plane sits 30 deg off horizontal when settled, and sweeps
 *          through 60 deg of arc when not
 *   height the tag plane is at 25.5 in. or 44.3 in. above the TILES, 18.8 in.
 *          apart, with nothing legitimate in between
 *
 * Both are measured relative to gravity rather than to the robot's own estimated
 * pose, so neither can be corrupted by the thing they are protecting. Requiring
 * them to agree is what makes a single bad solve fail closed.
 */
public class HiveTracker {

    private final AprilTagProcessor aprilTag;
    private final VisionPortal portal;

    /** One gated look at one CELL's cluster. */
    public static class Observation {
        public final HiveGeometry.Cell cell;
        public final HiveGeometry.Slot slot;
        public final double rangeIn;
        public final double bearingDeg;
        public final double elevationDeg;
        /** Tag plane angle from horizontal, gravity-referenced. */
        public final double tiltDeg;
        /** Tag plane height above the TILES, from camera height + geometry. */
        public final double heightIn;
        public final int percentClusterFound;
        public final boolean settled;
        /** Null when settled; otherwise why this observation was refused. */
        public final String rejectReason;

        Observation(HiveGeometry.Cell cell, HiveGeometry.Slot slot, double rangeIn,
                    double bearingDeg, double elevationDeg, double tiltDeg, double heightIn,
                    int percentClusterFound, boolean settled, String rejectReason) {
            this.cell = cell;
            this.slot = slot;
            this.rangeIn = rangeIn;
            this.bearingDeg = bearingDeg;
            this.elevationDeg = elevationDeg;
            this.tiltDeg = tiltDeg;
            this.heightIn = heightIn;
            this.percentClusterFound = percentClusterFound;
            this.settled = settled;
            this.rejectReason = rejectReason;
        }
    }

    public HiveTracker(HardwareMap hardwareMap, String webcamName) {
        aprilTag = new AprilTagProcessor.Builder()
                // The BIOBUZZ library is what teaches the solver that IDs 30-45 are
                // four-tag clusters. Without it you get four independent 3.25 in.
                // tags and a much worse pose.
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .build();
    }

    public VisionPortal getPortal() { return portal; }

    public void close() { portal.close(); }

    /**
     * @return one Observation per HIVE cluster currently in view, settled or not.
     *         Callers that only want fixes should filter on {@link Observation#settled};
     *         the bench OpMode wants the rejects too, which is why they are returned.
     */
    public List<Observation> observe() {
        List<Observation> out = new ArrayList<>();

        for (AprilTagDetection det : aprilTag.getDetections()) {
            // Individual member tags are reported alongside the fused cluster. Only
            // the cluster carries the 13 in. baseline that makes yaw trustworthy, so
            // single-tag detections are dropped rather than used as a weaker fix.
            if (!(det instanceof AprilTagClusterDetection)) continue;

            AprilTagClusterDetection cluster = (AprilTagClusterDetection) det;
            AprilTagPoseFtc pose = cluster.ftcPose;
            if (pose == null) continue;

            HiveGeometry.Cell cell = cellOf(cluster);
            if (cell == null) continue;

            double tilt   = tiltFromHorizontal(pose);
            double height = tagHeightAboveTiles(pose);

            String reject = null;
            if (cluster.percentClusterFound < HiveGeometry.MIN_CLUSTER_PERCENT) {
                reject = "partial cluster (" + cluster.percentClusterFound + "%)";
            }

            HiveGeometry.Slot slotByHeight = slotFromHeight(height);
            HiveGeometry.Slot slotByTilt   = slotFromTilt(tilt);

            if (reject == null && slotByHeight == HiveGeometry.Slot.UNKNOWN) {
                reject = String.format("height %.1f in matches neither slot", height);
            }
            if (reject == null && slotByTilt == HiveGeometry.Slot.UNKNOWN) {
                reject = String.format("tilt %.1f deg is mid-swing", tilt);
            }
            // The two tests are independent, so disagreement means one of them is
            // lying -- a bad solve, a mirrored pose, or a miscalibrated camera
            // pitch. Refuse rather than pick a winner.
            if (reject == null && slotByHeight != slotByTilt) {
                reject = "tilt and height disagree on slot";
            }

            HiveGeometry.Slot slot = (reject == null) ? slotByHeight : HiveGeometry.Slot.UNKNOWN;

            out.add(new Observation(cell, slot, pose.range, pose.bearing, pose.elevation,
                    tilt, height, cluster.percentClusterFound, reject == null, reject));
        }
        return out;
    }

    /**
     * Which CELL this cluster belongs to. The cluster metadata carries the member
     * IDs, so the first one identifies the cluster.
     */
    private static HiveGeometry.Cell cellOf(AprilTagClusterDetection cluster) {
        if (cluster.metadata == null || cluster.metadata.name == null) return null;
        for (HiveGeometry.Cell c : HiveGeometry.Cell.values()) {
            if (c.clusterName.equalsIgnoreCase(cluster.metadata.name)) return c;
        }
        return null;
    }

    /**
     * Tag plane angle from horizontal. ftcPose.pitch is relative to the camera, so
     * the camera's own mounting pitch has to come back out. On a flat FIELD the
     * robot contributes nothing; add IMU pitch here if you ever run on a ramp.
     */
    public static double tiltFromHorizontal(AprilTagPoseFtc pose) {
        return HiveGeometry.PITCH_SIGN * pose.pitch + HiveGeometry.CAMERA_PITCH_DEG;
    }

    /**
     * Tag plane height above the TILES. Uses only the camera's known mounting
     * height and the measured range/elevation -- deliberately independent of the
     * robot's estimated position, so it stays valid exactly when that estimate is
     * the thing in doubt.
     */
    public static double tagHeightAboveTiles(AprilTagPoseFtc pose) {
        double lookUpDeg = pose.elevation + HiveGeometry.CAMERA_PITCH_DEG;
        return HiveGeometry.CAMERA_HEIGHT_IN + pose.range * Math.sin(Math.toRadians(lookUpDeg));
    }

    public static HiveGeometry.Slot slotFromHeight(double heightIn) {
        if (Math.abs(heightIn - HiveGeometry.LOW_TAG_HEIGHT_IN) <= HiveGeometry.HEIGHT_TOL_IN) {
            return HiveGeometry.Slot.LOW;
        }
        if (Math.abs(heightIn - HiveGeometry.HIGH_TAG_HEIGHT_IN) <= HiveGeometry.HEIGHT_TOL_IN) {
            return HiveGeometry.Slot.HIGH;
        }
        return HiveGeometry.Slot.UNKNOWN;
    }

    public static HiveGeometry.Slot slotFromTilt(double tiltDeg) {
        if (Math.abs(tiltDeg - HiveGeometry.SETTLED_TILT_LOW_DEG) <= HiveGeometry.TILT_TOL_DEG) {
            return HiveGeometry.Slot.LOW;
        }
        if (Math.abs(tiltDeg - HiveGeometry.SETTLED_TILT_HIGH_DEG) <= HiveGeometry.TILT_TOL_DEG) {
            return HiveGeometry.Slot.HIGH;
        }
        return HiveGeometry.Slot.UNKNOWN;
    }
}
