package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Tank TeleOp for the goBILDA BIOBUZZ StarterBot / kitbot -- one motor per side.
 *
 * ONE STICK DRIVES. A skid-steer cannot strafe, so instead of "left stick forward,
 * right stick turn", the LEFT stick does everything: up/down is forward/back, and
 * left/right turns. The right stick is unused.
 *
 *   left stick up/down    forward / back
 *   left stick left/right turn
 *   right trigger         precision creep -- scales the drive down for lining up
 *   right bumper          intake (roller + servos) forward, AND windmill forward
 *   left bumper           intake (roller + servos) forward only
 *   square                intake ROLLER reverse + windmill reverse; servos stay still
 *   dpad up / down        launcher speed -/+  (launcher runs in MANUAL, always on)
 *
 * THE LAUNCHER IS ALWAYS RUNNING. There is no arm button: from START it spins at the
 * manual speed (ShooterControls.DEFAULT_MANUAL_VELOCITY), so the driver only tunes it
 * with the dpad and feeds with the intake. Because it is manual, the Limelight is not
 * needed and this OpMode does not use the camera at all.
 *
 * WINDMILL WITH THE INTAKE. The windmill is the feed servo that pushes balls into the
 * launcher. It runs forward with the right bumper (intake + feed together, so one press
 * loads and fires) and in reverse with square (clears a jam). The left bumper runs the
 * intake WITHOUT the windmill, so a driver can load without firing.
 */
@TeleOp(name = "2. Tank TeleOp (kitbot)", group = "Drive")
public class TankTeleOp extends LinearOpMode {

    /** Gamepad sticks rarely read exactly zero when you let go. Below this is "not touched". */
    private static final double DEADBAND = 0.05;

    /** Multiplier at full precision trigger. */
    private static final double CREEP_SCALE = 0.30;

    /** Top speed as a fraction of full power. 1.0 is full; lower it to slow the robot. */
    private static final double DRIVE_GAIN = 1.0;

    /** Manual launcher speed step for the dpad up/down buttons. */
    private static final double MANUAL_STEP = ShooterControls.MANUAL_STEP;

    @Override
    public void runOpMode() {
        TankDrivebase drive = new TankDrivebase(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        ShooterControls shooter = new ShooterControls(new Shooter(hardwareMap));

        // The launcher is always on and always manual -- no camera, no arm button.
        shooter.setManual(true);
        shooter.setArmed(true);

        boolean upPrev = false, downPrev = false;

        telemetry.addLine("Ready. Point the robot downfield before START.");
        telemetry.addData("launcher", shooter.shooter().hasLauncher() ? "OK" : "MISSING");
        telemetry.addData("windmill", shooter.shooter().hasWindmill() ? "OK" : "MISSING");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // --- drive: arcade, no strafe (a skid-steer cannot slide). Turn comes
            // from EITHER stick's x, so a driver can steer with whichever thumb is
            // free -- the left stick alone works, and so does the right. ---
            double forward = square(deadband(-gamepad1.left_stick_y));
            double turn    = square(deadband(gamepad1.left_stick_x))
                           + square(deadband(gamepad1.right_stick_x));
            turn = clamp(turn, -1.0, 1.0);

            double scale = DRIVE_GAIN * (1.0 - (1.0 - CREEP_SCALE) * gamepad1.right_trigger);
            drive.driveRobotCentric(forward * scale, turn * scale);

            // --- intake, servos and windmill, from the bumpers and square ---
            // Computed separately because square reverses only the roller and the
            // windmill -- the corner servos hold still, so reversing does not fight
            // the roller.
            double intakeMotor = 0.0, intakeServos = 0.0, windmill = 0.0;
            if (gamepad1.square) {
                intakeMotor = -1.0;   // reverse the roller to unjam / outake
                windmill    = -1.0;   // push the ball back out of the launcher
                // servos stay 0: they would fight the roller if reversed
            } else if (gamepad1.right_bumper) {
                intakeMotor = 1.0;
                intakeServos = 1.0;
                windmill = 1.0;       // load and fire in one press
            } else if (gamepad1.left_bumper) {
                intakeMotor = 1.0;
                intakeServos = 1.0;
                // no windmill: load without firing
            }
            intake.setMotor(intakeMotor);
            intake.setServos(intakeServos);
            shooter.shooter().setFeed(windmill);

            // --- launcher: always spinning in MANUAL; dpad tunes the speed ---
            if (gamepad1.dpad_up   && !upPrev)   shooter.nudgeManualVelocity(+MANUAL_STEP);
            if (gamepad1.dpad_down && !downPrev) shooter.nudgeManualVelocity(-MANUAL_STEP);
            upPrev = gamepad1.dpad_up; downPrev = gamepad1.dpad_down;
            shooter.update(null);   // manual: no range needed

            // Square reverses the launcher too, for the whole time it is held. This is
            // set AFTER update() so it wins over the manual speed. The sign matters:
            // setVelocity takes negatives, which is what makes the wheel run backwards.
            if (gamepad1.square) {
                shooter.shooter().setVelocity(-shooter.getManualVelocity());
            }

            // --- telemetry ---
            telemetry.addData("drive", "fwd %+.2f  turn %+.2f", forward * scale, turn * scale);
            telemetry.addData("intake", "motor %+.0f  servos %+.0f  windmill %+.0f",
                    intakeMotor, intakeServos, windmill);
            telemetry.addData("launcher", "asked %.0f  actual %.0f ticks/s  %s",
                    shooter.getCommandedVelocity(), shooter.getVelocity(),
                    shooter.isSpunUp() ? "READY" : "spinning up");
            telemetry.addLine();
            telemetry.addLine("Left stick drives.  RB intake+fire  LB intake  square reverse");
            telemetry.addLine("dpad up/down = launcher speed");
            telemetry.update();
        }

        drive.stop();
        intake.stop();
        shooter.stop();
    }

    private static double deadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : value;
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }

    private static double clamp(double v, double lo, double hi) {
        if (v > hi) return hi;
        if (v < lo) return lo;
        return v;
    }
}
