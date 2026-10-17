package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Tank TeleOp for the RED alliance.
 *
 * All of the behaviour is in {@link TankTeleOpBase}; the only thing this file decides
 * is which tag cluster is our goal. Read the base class header for why the alliance has
 * to be picked here rather than worked out from the camera.
 *
 * RED_FAR is the "RED SCORING" cluster, ids 30-33 (HiveGeometry.Cell).
 */
@TeleOp(name = "2a. Tank TeleOp (RED)", group = "Drive")
public class TankTeleOpRed extends TankTeleOpBase {
    @Override
    protected HiveGeometry.Cell scoringCell() {
        return HiveGeometry.Cell.RED_FAR;
    }
}
