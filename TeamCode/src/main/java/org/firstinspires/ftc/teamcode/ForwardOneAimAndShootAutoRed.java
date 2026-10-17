package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Autonomous for the RED alliance: one square forward, aim at the RED goal, shoot.
 *
 * All of the behaviour is in {@link ForwardOneAimAndShootAutoBase}; the only thing this
 * file decides is which tag cluster is our goal. Read the base class header for why the
 * alliance has to be picked here rather than worked out from the camera.
 *
 * RED_FAR is the "RED SCORING" cluster, ids 30-33 (HiveGeometry.Cell).
 */
@Autonomous(name = "9a. Auto: 1 Square, Aim & Shoot (RED)", group = "Auto")
public class ForwardOneAimAndShootAutoRed extends ForwardOneAimAndShootAutoBase {
    @Override
    protected HiveGeometry.Cell scoringCell() {
        return HiveGeometry.Cell.RED_FAR;
    }
}
