package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Shoot, then drive -- the RED alliance.
 *
 * All of the behaviour is in {@link ShootThenDriveAutoBase}, so red and blue run exactly
 * the same code; the only thing this file decides is the alliance. Read the base class
 * header for where to start the robot, and why red uses the same turns as blue.
 */
@Autonomous(name = "12. Auto: Shoot, Then Drive (Red)", group = "Auto")
public class ShootThenDriveAutoRed extends ShootThenDriveAutoBase {
    @Override
    protected HiveGeometry.Alliance alliance() {
        return HiveGeometry.Alliance.RED;
    }
}
