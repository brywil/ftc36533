package org.firstinspires.ftc.teamcode;

import java.util.List;

/**
 * Aim geometry for one HIVE CELL.
 *
 * The four tags of a CELL are a rigid row, and the SDK's BIOBUZZ cluster metadata gives
 * their offsets from the sticker centre: x = -6.5, -2.75, +2.75, +6.5 in. Knowing those
 * offsets means ONE visible tag is enough to estimate the centre of the sticker:
 * correct that tag's tx by atan(dx / range). The old code averaged the raw tx of
 * whichever members happened to be visible, so seeing only the left or right member
 * pulled the aim toward that member -- exactly what the team saw.
 *
 * The sign of the cluster's +x in the image is learned from any two members of the
 * same CELL: their ID order says which way +x runs, their tx order says which way the
 * image runs. Until such a pair is seen, DEFAULT_X_SIGN is used.
 */
final class HiveAim {

    /** +1 if increasing tag ID appears to the right in the image; -1 if reversed. */
    private static final double DEFAULT_X_SIGN = 1.0;

    private double xSign = DEFAULT_X_SIGN;
    private boolean signKnown = false;

    /** Learn the x-axis sign from any two tags that belong to the same CELL. */
    void learnSign(List<LimelightHiveTracker.Tag> tags) {
        for (int i = 0; i < tags.size(); i++) {
            for (int j = i + 1; j < tags.size(); j++) {
                LimelightHiveTracker.Tag a = tags.get(i);
                LimelightHiveTracker.Tag b = tags.get(j);
                if (a.cell == null || a.cell != b.cell) continue;
                double dx = HiveGeometry.clusterMemberX(b.id)
                          - HiveGeometry.clusterMemberX(a.id);
                double dtx = b.txDeg - a.txDeg;
                if (Math.abs(dx) < 0.5 || Math.abs(dtx) < 0.5) continue;
                xSign = (dx * dtx >= 0.0) ? 1.0 : -1.0;
                signKnown = true;
                return;
            }
        }
    }

    /**
     * Bearing to the centre of the CELL these tags belong to, in degrees, or null when
     * the list is empty. Every visible member is corrected by its own known offset at
     * its own range, then the corrected bearings are averaged. That is what makes one
     * tag enough, and makes two tags agree instead of pulling toward one of them.
     */
    Double centerBearing(List<LimelightHiveTracker.Tag> tags) {
        double sum = 0.0;
        int n = 0;
        for (LimelightHiveTracker.Tag t : tags) {
            double dx = HiveGeometry.clusterMemberX(t.id);
            double bearing = t.txDeg;
            if (!Double.isNaN(t.rangeIn) && t.rangeIn > 1.0 && Math.abs(dx) > 0.01) {
                // A tag left of centre means the centre is to its right, hence subtract.
                bearing -= Math.toDegrees(Math.atan2(dx * xSign, t.rangeIn));
            }
            sum += bearing;
            n++;
        }
        return n == 0 ? null : sum / n;
    }

    /**
     * Which of our HIVE's two CELLs is UP, or null when the tags cannot say. Pass the
     * tags grouped by cell: primary is our scoring cell, opposite is the other end of
     * the same HIVE (the AUDIENCE side). A partial cluster is enough.
     */
    HiveGeometry.Cell highCell(HiveGeometry.Cell primary,
                               List<LimelightHiveTracker.Tag> primaryTags,
                               List<LimelightHiveTracker.Tag> oppositeTags) {
        HiveGeometry.Slot p = LimelightHiveTracker.knownSlot(primaryTags);
        HiveGeometry.Slot o = LimelightHiveTracker.knownSlot(oppositeTags);
        if (p == HiveGeometry.Slot.HIGH && o != HiveGeometry.Slot.HIGH) return primary;
        if (o == HiveGeometry.Slot.HIGH && p != HiveGeometry.Slot.HIGH) return primary.opposite();
        return null;
    }

    /** True once a same-CELL pair has settled the +x direction. */
    boolean isSignKnown() { return signKnown; }
}
