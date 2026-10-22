package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Shoot, then drive -- the BLUE alliance.
 *
 * All of the behaviour is in {@link ShootThenDriveAutoBase}, so red and blue run exactly
 * the same code; the only thing this file decides is the alliance. Read the base class
 * header for where to start the robot, and why red uses the same turns as blue.
 */
@Autonomous(name = "11. Auto: Shoot, Then Drive (Blue)", group = "Auto")
public class ShootThenDriveAutoBlue extends ShootThenDriveAutoBase {
    @Override
    protected HiveGeometry.Alliance alliance() {
        return HiveGeometry.Alliance.BLUE;
    }
}
