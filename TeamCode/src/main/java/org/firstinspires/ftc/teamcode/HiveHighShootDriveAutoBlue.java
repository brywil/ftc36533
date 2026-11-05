package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Support-blue version of {@link HiveHighShootDriveAutoBase}: watches the BLUE AUDIENCE
 * cluster (ids 38-41) for the HIGH slot, then shoots and drives.
 *
 * WHY NOT BLUE SCORING (42-45). The field is point-symmetric, so the two alliances stage
 * the HIVE the opposite way round: RED starts with its AUDIENCE-side CELL up, BLUE starts
 * with its REAR/scoring-side CELL up (manual Fig 10-2). This robot starts with BLUE
 * SCORING already up. The partner is expected to tip the HIVE, which puts BLUE SCORING
 * down and BLUE AUDIENCE up -- so BLUE AUDIENCE is the CELL this robot can score into
 * after the tip. Watching BLUE SCORING would fire before any tip, at the wrong CELL.
 *
 * Everything else -- the wait, the shot, the movement -- lives in the base class.
 */
@Autonomous(name = "14. Auto: HIVE High, Shoot, Drive (Support Blue)", group = "Auto")
public class HiveHighShootDriveAutoBlue extends HiveHighShootDriveAutoBase {
    @Override
    protected HiveGeometry.Cell scoringCell() {
        return HiveGeometry.Cell.BLUE_AUDIENCE;
    }
}
