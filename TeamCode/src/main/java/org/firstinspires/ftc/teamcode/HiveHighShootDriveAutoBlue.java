package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Support-blue version of {@link HiveHighShootDriveAutoBase}: watches the BLUE SCORING
 * cluster (ids 42-45) for the HIGH slot, then shoots and drives.
 *
 * Everything else -- the wait, the shot, the movement -- lives in the base class. This
 * file only answers "which cluster is our goal", the same split as
 * {@link ForwardOneAimAndShootAutoBlue}.
 */
@Autonomous(name = "14. Auto: HIVE High, Shoot, Drive (Support Blue)", group = "Auto")
public class HiveHighShootDriveAutoBlue extends HiveHighShootDriveAutoBase {
    @Override
    protected HiveGeometry.Cell scoringCell() {
        return HiveGeometry.Cell.BLUE_FAR;
    }
}
