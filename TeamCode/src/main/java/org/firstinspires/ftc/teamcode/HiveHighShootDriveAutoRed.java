package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Support-red version of {@link HiveHighShootDriveAutoBase}: watches the RED SCORING
 * cluster (ids 30-33) for the HIGH slot, then shoots and drives.
 *
 * WHY RED SCORING IS RIGHT HERE. The field is point-symmetric, so RED starts with its
 * AUDIENCE-side CELL up and its REAR/scoring-side CELL down (manual Fig 10-2). This robot
 * starts with RED SCORING down; the partner tips the HIVE, RED SCORING comes up, and this
 * robot scores into it. That is the mirror of the BLUE support auto, which watches BLUE
 * AUDIENCE for the same reason.
 *
 * Everything else -- the wait, the shot, the movement -- lives in the base class.
 */
@Autonomous(name = "13. Auto: HIVE High, Shoot, Drive (Support Red)", group = "Auto")
public class HiveHighShootDriveAutoRed extends HiveHighShootDriveAutoBase {
    @Override
    protected HiveGeometry.Cell scoringCell() {
        return HiveGeometry.Cell.RED_FAR;
    }
}
