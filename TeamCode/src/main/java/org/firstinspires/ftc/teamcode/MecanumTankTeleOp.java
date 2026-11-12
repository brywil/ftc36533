package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Tank-style driving for the four-motor mecanum robot.
 *
 * The chassis is mecanum, but this OpMode drives it like a tank: no sliding
 * sideways and no field-centric mode, because a driver who wants tank controls wants
 * the robot to go where the nose points. The four motors are paired into a left and
 * a right side by {@link MecanumTankDrivebase}, so both wheels on a side always agree.
 *
 * Two control schemes, same buttons as the kitbot:
 *
 *   ARCADE (default)
 *     left stick up/down     forward / back
 *     left stick left/right  turn
 *     right stick left/right turn (same thing; either thumb works)
 *
 *   TANK (press BACK to toggle)
 *     left stick up/down     drives the left side
 *     right stick up/down    drives the right side
 *
 * Always:
 *     right trigger          precision creep -- scales everything down
 *
 * It is drive-only on purpose: no intake, launcher, aim or camera, so it runs on a
 * bare chassis whose Robot Configuration has nothing but drive motors. If the
 * OpMode says it cannot find the motors, run "0. Hardware Check" (or look at the
 * configuration) and check the four names it lists in its message.
 */
@TeleOp(name = "2c. Mecanum Tank TeleOp", group = "Drive")
public class MecanumTankTeleOp extends LinearOpMode {

    /** Gamepad sticks rarely read exactly zero; below this counts as "not touched". */
    private static final double DEADBAND = 0.05;

    /** Multiplier at full precision trigger. */
    private static final double CREEP_SCALE = 0.30;

    @Override
    public void runOpMode() {
        MecanumTankDrivebase drive = new MecanumTankDrivebase(hardwareMap);

        telemetry.addData("drive", drive.describe());
        if (!drive.isReady()) {
            telemetry.addLine();
            telemetry.addLine("No drive motors found. Expected either:");
            telemetry.addLine("  4 motors: front_left, front_right,");
            telemetry.addLine("            back_left, back_right");
            telemetry.addLine("  2 motors: left_drive, right_drive");
            telemetry.addLine();
            telemetry.addLine("Run \"0. Hardware Check\" to see what names");
            telemetry.addLine("the Robot Configuration actually has.");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addLine("WHEELS OFF THE GROUND for the first test.");
        telemetry.addLine("Left stick drives. BACK toggles ARCADE / TANK.");
        telemetry.addLine("Right trigger = precision creep.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean tankSteering = false;
        boolean backPrev = false;

        while (opModeIsActive()) {
            if (gamepad1.back && !backPrev) tankSteering = !tankSteering;
            backPrev = gamepad1.back;

            double scale = 1.0 - (1.0 - CREEP_SCALE) * gamepad1.right_trigger;

            if (tankSteering) {
                // Each stick owns its side. Square the input so small pushes are gentle.
                double left  = square(deadband(-gamepad1.left_stick_y))  * scale;
                double right = square(deadband(-gamepad1.right_stick_y)) * scale;
                drive.driveTank(left, right);
            } else {
                // Squaring the forward/turn inputs makes small pushes gentler while a
                // full push still gives full power -- fine control near centre.
                double forward = square(deadband(-gamepad1.left_stick_y)) * scale;
                double turn = clamp(
                        square(deadband(gamepad1.left_stick_x))
                      + square(deadband(gamepad1.right_stick_x)), -1.0, 1.0) * scale;
                drive.driveArcade(forward, turn);
            }

            telemetry.addData("drive", drive.describe());
            telemetry.addData("mode", tankSteering ? "TANK (each stick = one side)"
                                                   : "ARCADE (left stick)");
            telemetry.addData("sides", "left %+.2f  right %+.2f",
                    drive.leftPower(), drive.rightPower());
            telemetry.addData("precision", "%.0f%%", scale * 100);
            telemetry.addLine();
            telemetry.addLine("BACK: arcade / tank    RT: slow down");
            telemetry.update();
        }

        drive.stop();
    }

    private static double deadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : value;
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
