package org.firstinspires.ftc.teamcode;

/**
 * The settled-gate decision, shared by every HIVE tracker.
 *
 * Two trackers read the same physical HIVE through different hardware -- the USB
 * webcam through the FTC SDK's AprilTagProcessor (HiveTracker) and the Limelight
 * 3A through its fiducial pipeline (LimelightHiveTracker). The hardware differs;
 * the physics does not. Both end up with a tilt and a height and have to make the
 * same call, so the call lives here once rather than being copied.
 *
 * The gate is unchanged from the original HiveTracker (commit da12ef9):
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
public final class HiveGate {

    private HiveGate() {}

    /** The gate's verdict for one look at one cluster. */
    public static final class Verdict {
        public final HiveGeometry.Slot slot;
        public final boolean settled;
        /** Null when settled; otherwise why this observation was refused. */
        public final String rejectReason;

        Verdict(HiveGeometry.Slot slot, boolean settled, String rejectReason) {
            this.slot = slot;
            this.settled = settled;
            this.rejectReason = rejectReason;
        }
    }

    /**
     * @param memberIdsFound how many of the cluster's four member tags were seen;
     *                       callers pass -1 when the hardware reports no such count
     *                       and the check is skipped.
     */
    public static Verdict evaluate(double tiltDeg, double heightIn, int memberIdsFound) {
        String reject = null;

        if (memberIdsFound >= 0 && memberIdsFound < HiveGeometry.CLUSTER_MEMBER_COUNT) {
            reject = "partial cluster (" + memberIdsFound + "/" + HiveGeometry.CLUSTER_MEMBER_COUNT + ")";
        }

        HiveGeometry.Slot slotByHeight = slotFromHeight(heightIn);
        HiveGeometry.Slot slotByTilt   = slotFromTilt(tiltDeg);

        if (reject == null && slotByHeight == HiveGeometry.Slot.UNKNOWN) {
            reject = String.format("height %.1f in matches neither slot", heightIn);
        }
        if (reject == null && slotByTilt == HiveGeometry.Slot.UNKNOWN) {
            reject = String.format("tilt %.1f deg is mid-swing", tiltDeg);
        }
        // The two tests are independent, so disagreement means one of them is
        // lying -- a bad solve, a mirrored pose, or a miscalibrated camera pitch.
        // Refuse rather than pick a winner.
        if (reject == null && slotByHeight != slotByTilt) {
            reject = "tilt and height disagree on slot";
        }

        HiveGeometry.Slot slot = (reject == null) ? slotByHeight : HiveGeometry.Slot.UNKNOWN;
        return new Verdict(slot, reject == null, reject);
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
