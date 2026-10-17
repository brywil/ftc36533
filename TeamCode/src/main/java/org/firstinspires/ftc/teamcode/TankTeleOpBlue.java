package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Tank TeleOp for the BLUE alliance.
 *
 * All of the behaviour is in {@link TankTeleOpBase}; the only thing this file decides
 * is which tag cluster is our goal. Read the base class header for why the alliance has
 * to be picked here rather than worked out from the camera.
 *
 * BLUE_FAR is the "BLUE SCORING" cluster, ids 42-45 (HiveGeometry.Cell).
 */
@TeleOp(name = "2b. Tank TeleOp (BLUE)", group = "Drive")
public class TankTeleOpBlue extends TankTeleOpBase {
    @Override
    protected HiveGeometry.Cell scoringCell() {
        return HiveGeometry.Cell.BLUE_FAR;
    }
}
