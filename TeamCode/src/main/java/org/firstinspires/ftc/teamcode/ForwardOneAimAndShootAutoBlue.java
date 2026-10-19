package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Autonomous for the BLUE alliance: one square forward, aim at the BLUE goal, shoot.
 *
 * All of the behaviour is in {@link ForwardOneAimAndShootAutoBase}; the only thing this
 * file decides is which tag cluster is our goal. Read the base class header for why the
 * alliance has to be picked here rather than worked out from the camera.
 *
 * BLUE_FAR is the "BLUE SCORING" cluster, ids 42-45 (HiveGeometry.Cell).
 */
@Autonomous(name = "9blue. Auto: 2 Squares, Aim, Shoot, Park", group = "Auto")
public class ForwardOneAimAndShootAutoBlue extends ForwardOneAimAndShootAutoBase {
    @Override
    protected HiveGeometry.Cell scoringCell() {
        return HiveGeometry.Cell.BLUE_FAR;
    }
}
