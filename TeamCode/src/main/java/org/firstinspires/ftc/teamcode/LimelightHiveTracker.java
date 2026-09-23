package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Reads the BIOBUZZ HIVE AprilTags through the Limelight 3A, instead of through a
 * USB webcam and the FTC SDK's AprilTagProcessor.
 *
 * The sibling of {@link HiveTracker}. The settled-gate physics is identical and
 * lives in {@link HiveGate}; what changes is only where the numbers come from.
 *
 *   webcam (HiveTracker)          Limelight (this)
 *   ---------------------------   ------------------------------------------
 *   USB webcam + SDK processor    Limelight 3A, fiducial pipeline
 *   native 4-tag cluster fusion   no fusion -- this groups the member IDs
 *   ftcPose range/bearing/elev    camera-space Pose3D per tag
 *   percentClusterFound           count of the four member IDs seen
 *
 * What the Limelight does NOT give for free is the cluster fusion, so this class
 * does that grouping itself: the four tags of a CELL are a rigid printed sticker,
 * so grouping by ID range (see {@link HiveGeometry.Cell}) recovers the cluster and
 * averaging over its members recovers a stable pose. That is strictly weaker than
 * the SDK's fusion -- there is no 13 in. baseline solve, the members are treated
 * as independent -- but it is enough for the gate, which only needs range and
 * angles, not a yaw you would steer by.
 *
 * No field map (.fmap) is used or needed. Tag field poses are still unmeasured
 * (HiveGeometry.SLOT_POSES_MEASURED is false), so this reports range, bearing,
 * slot and a settled verdict, exactly as the proof of concept does -- not a field
 * pose.
 *
 * ---------------------------------------------------------------------------
 * SIGN CONVENTIONS. The camera-space pose comes back in the OpenCV convention
 * (x right, y down, z forward), and the SDK hands back yaw/pitch/roll. Two of the
 * signs below are assumptions that only the bench can confirm -- they are called
 * out where used, and LimelightHiveBenchOpMode prints everything needed to check
 * them.
 * ---------------------------------------------------------------------------
 */
public class LimelightHiveTracker {

    /** Reject a result older than this; a stale frame aims at where the HIVE WAS. */
    private static final long MAX_STALENESS_MS = 200;

    private final Limelight3A limelight;

    /**
     * @param limelightName Robot Configuration name of the Limelight (Ethernet Device).
     * @param pipelineIndex the fiducial (AprilTag) pipeline to switch to. Limelight
     *                      indexes pipelines 0..9; the SDK has no switch-by-name.
     */
    public LimelightHiveTracker(HardwareMap hardwareMap, String limelightName, int pipelineIndex) {
        limelight = hardwareMap.get(Limelight3A.class, limelightName);
        limelight.pipelineSwitch(pipelineIndex);
        limelight.setPollRateHz(100);
    }

    public Limelight3A getLimelight() { return limelight; }

    public void start() { limelight.start(); }
    public void stop()  { limelight.stop();  }
    public void close() { limelight.stop();  }

    /** One detected tag, HIVE tag or not. Angles in degrees, lengths in inches. */
    public static class Tag {
        public final int id;
        public final String family;
        /** The CELL this tag belongs to, or null when it is not a BIOBUZZ HIVE tag. */
        public final HiveGeometry.Cell cell;
        /** Crosshair-relative angles, straight from the Limelight. */
        public final double txDeg, tyDeg;
        /** Distance to the tag. */
        public final double rangeIn;
        /** Tag plane angle from horizontal, gravity-referenced; NaN if no 3D solve. */
        public final double tiltDeg;
        /** Tag centre height above the TILES; NaN if no 3D solve. */
        public final double heightIn;

        Tag(int id, String family, HiveGeometry.Cell cell, double txDeg, double tyDeg,
            double rangeIn, double tiltDeg, double heightIn) {
            this.id = id;
            this.family = family;
            this.cell = cell;
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.rangeIn = rangeIn;
            this.tiltDeg = tiltDeg;
            this.heightIn = heightIn;
        }

        /** True when the Limelight could solve this tag in 3D (needs 3D solve on). */
        public boolean hasPose() { return !Double.isNaN(tiltDeg); }
    }

    /** One gated look at one CELL's cluster, plus the member tags it was built from. */
    public static class Observation {
        public final HiveGeometry.Cell cell;
        public final HiveGeometry.Slot slot;
        public final double rangeIn;
        public final double bearingDeg;
        public final double elevationDeg;
        /** Mean tag plane angle from horizontal over the members seen. */
        public final double tiltDeg;
        /** Mean tag centre height above the TILES over the members seen. */
        public final double heightIn;
        /** How many of the cluster's four member IDs were seen. */
        public final int membersFound;
        public final boolean settled;
        /** Null when settled; otherwise why this observation was refused. */
        public final String rejectReason;
        /** The member tags, in the order detected. Never empty. */
        public final List<Tag> members;

        Observation(HiveGeometry.Cell cell, HiveGeometry.Slot slot, double rangeIn,
                    double bearingDeg, double elevationDeg, double tiltDeg, double heightIn,
                    int membersFound, boolean settled, String rejectReason, List<Tag> members) {
            this.cell = cell;
            this.slot = slot;
            this.rangeIn = rangeIn;
            this.bearingDeg = bearingDeg;
            this.elevationDeg = elevationDeg;
            this.tiltDeg = tiltDeg;
            this.heightIn = heightIn;
            this.membersFound = membersFound;
            this.settled = settled;
            this.rejectReason = rejectReason;
            this.members = members;
        }
    }

    /**
     * Every tag the Limelight currently sees, recognized and converted, including
     * tags that are not part of a HIVE cluster (their {@link Tag#cell} is null).
     * This is the raw recognition view; use {@link #observe()} for the gate.
     */
    public List<Tag> recognize() {
        List<Tag> out = new ArrayList<>();
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return out;
        if (result.getStaleness() > MAX_STALENESS_MS) return out;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return out;

        for (LLResultTypes.FiducialResult fr : fiducials) {
            out.add(toTag(fr));
        }
        return out;
    }

    /**
     * Range in inches to the nearest tag in view, 3D-solved or not, HIVE tag or
     * not. Null when there is no tag with a usable 3D pose. Convenience for callers
     * that only want a distance -- a shooter ranging on a target tag, say -- rather
     * than the HIVE cluster machinery.
     */
    public Double nearestRangeIn() {
        Double best = null;
        for (Tag t : recognize()) {
            if (Double.isNaN(t.rangeIn)) continue;
            if (best == null || t.rangeIn < best) best = t.rangeIn;
        }
        return best;
    }

    /**
     * One Observation per HIVE cluster currently in view, settled or not.
     * Callers that only want fixes should filter on {@link Observation#settled};
     * the bench OpMode wants the rejects too, which is why they are returned.
     */
    public List<Observation> observe() {
        Map<HiveGeometry.Cell, List<Tag>> byCell = new EnumMap<>(HiveGeometry.Cell.class);
        for (Tag t : recognize()) {
            if (t.cell == null) continue;
            List<Tag> group = byCell.get(t.cell);
            if (group == null) {
                group = new ArrayList<>();
                byCell.put(t.cell, group);
            }
            group.add(t);
        }

        List<Observation> out = new ArrayList<>();
        for (Map.Entry<HiveGeometry.Cell, List<Tag>> e : byCell.entrySet()) {
            out.add(gate(e.getKey(), e.getValue()));
        }
        return out;
    }

    private static Observation gate(HiveGeometry.Cell cell, List<Tag> members) {
        int found = members.size();

        // Members of one sticker are coplanar, so averaging is the cheap stand-in
        // for cluster fusion: it cancels per-tag solve noise instead of trusting
        // whichever tag happened to be detected first.
        double sumRange = 0, sumTilt = 0, sumHeight = 0;
        int posed = 0;
        Tag rep = members.get(0);
        for (Tag t : members) {
            sumRange += t.rangeIn;
            if (t.hasPose()) {
                sumTilt += t.tiltDeg;
                sumHeight += t.heightIn;
                posed++;
            }
            if (t.rangeIn < rep.rangeIn) rep = t;
        }
        double avgRange = sumRange / found;
        // With no member 3D-solved the means are undefined; a NaN tilt makes the
        // gate say so, which is the honest answer rather than a silent guess.
        double avgTilt   = posed > 0 ? sumTilt / posed : Double.NaN;
        double avgHeight = posed > 0 ? sumHeight / posed : Double.NaN;

        HiveGate.Verdict verdict = HiveGate.evaluate(avgTilt, avgHeight, found);

        // bearing/elevation come from the most head-on member (smallest range),
        // and ty is flipped so elevation is positive-up like the webcam path.
        double bearing = rep.txDeg;
        double elevation = -rep.tyDeg;

        return new Observation(cell, verdict.slot, avgRange, bearing, elevation,
                avgTilt, avgHeight, found, verdict.settled, verdict.rejectReason, members);
    }

    private static Tag toTag(LLResultTypes.FiducialResult fr) {
        int id = fr.getFiducialId();
        HiveGeometry.Cell cell = HiveGeometry.Cell.fromTagId(id);
        double tx = fr.getTargetXDegrees();
        double ty = fr.getTargetYDegrees();

        double rangeIn = Double.NaN, tilt = Double.NaN, height = Double.NaN;
        Pose3D cameraSpace = fr.getTargetPoseCameraSpace();
        if (cameraSpace != null && cameraSpace.getPosition() != null) {
            org.firstinspires.ftc.robotcore.external.navigation.Position p =
                    cameraSpace.getPosition();
            double norm = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            DistanceUnit u = (p.unit != null) ? p.unit : DistanceUnit.METER;
            rangeIn = u.toInches(norm);

            if (cameraSpace.getOrientation() != null) {
                // The tag plane's pitch, relative to the camera. This is the
                // LIMELIGHT_TILT_SIGN assumption: raising the far edge of the tag
                // must move this number in one consistent direction. Confirm on
                // the bench and flip the constant if wrong.
                double tagPitch = cameraSpace.getOrientation().getPitch(AngleUnit.DEGREES);
                tilt = HiveGeometry.LIMELIGHT_TILT_SIGN * tagPitch
                     + HiveGeometry.LIMELIGHT_PITCH_DEG;
            }

            // Same construction the webcam path uses: camera mounting height plus
            // the range resolved onto the vertical. elevation is positive-up, and
            // the camera's own pitch has to come back out the same way it went in.
            if (!Double.isNaN(rangeIn)) {
                double lookUpDeg = -ty + HiveGeometry.LIMELIGHT_PITCH_DEG;
                height = HiveGeometry.LIMELIGHT_HEIGHT_IN
                       + rangeIn * Math.sin(Math.toRadians(lookUpDeg));
            }
        }

        return new Tag(id, fr.getFamily(), cell, tx, ty, rangeIn, tilt, height);
    }

    /** One-line description of a tag, for telemetry. */
    public static String describe(Tag t) {
        return String.format(Locale.US, "id %d %s%s  tx %+6.1f ty %+6.1f %s",
                t.id, t.family,
                t.cell != null ? " (" + t.cell.name() + ")" : " (not a HIVE tag)",
                t.txDeg, t.tyDeg,
                Double.isNaN(t.rangeIn) ? "no 3D pose" : String.format(Locale.US, "r %.1f in", t.rangeIn));
    }
}
