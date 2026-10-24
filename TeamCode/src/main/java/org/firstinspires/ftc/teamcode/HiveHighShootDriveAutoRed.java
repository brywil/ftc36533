package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Support-red version of {@link HiveHighShootDriveAutoBase}: watches the RED SCORING
 * cluster (ids 30-33) for the HIGH slot, then shoots and drives.
 *
 * Everything else -- the wait, the shot, the movement -- lives in the base class. This
 * file only answers "which cluster is our goal", the same split as
 * {@link ForwardOneAimAndShootAutoRed}.
 */
@Autonomous(name = "13. Auto: HIVE High, Shoot, Drive (Support Red)", group = "Auto")
public class HiveHighShootDriveAutoRed extends HiveHighShootDriveAutoBase {
    @Override
    protected HiveGeometry.Cell scoringCell() {
        return HiveGeometry.Cell.RED_FAR;
    }
}
