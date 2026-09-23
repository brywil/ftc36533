package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Locale;

/**
 * Points the robot at an AprilTag by itself. The driver still drives; this only
 * takes over the TURN axis, keeping the target centered while the driver lines up.
 *
 * Run it on its own first, before wiring aim into shooting. The question it answers
 * is the one that cannot be reasoned out: does the robot turn the right way? See
 * AutoAim's header for the tuning order -- confirm direction, then KP, then KD.
 *
 * Like "1. Simple Drive", it finds whatever motors exist -- four (mecanum) or two
 * (tank) -- so it works on both robots without the IMU, which aiming does not need.
 *
 * Controls:
 *   left stick        drive (and slide, on mecanum) -- always the driver's
 *   A                 aim on / off
 *   right stick L/R   turn by hand (only while aim is OFF)
 *   B                 hold for a manual turn override while aim is on
 *
 * The moment aim is on, the driver's forward/slide still work; only turning is taken
 * over. Turn aim off and the robot is exactly as it was.
 */
@TeleOp(name = "7. Auto Aim (AprilTag)", group = "Drive")
public class AutoAimOpMode extends LinearOpMode {

    /** Must match the Robot Configuration name of the Limelight. */
    private static final String LIMELIGHT_NAME = "limelight";

    /** The fiducial pipeline you configured on the camera (0..9). */
    private static final int FIDUCIAL_PIPELINE = 1;

    /** Pin aim to one tag ID, or -1 to aim at the nearest tag in view. */
    private static final int TARGET_TAG_ID = -1;

    /** Drive speed cap, matching Simple Drive -- fast robots break things. */
    private static final double DRIVE_SPEED = 0.5;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private boolean fourWheel;

    @Override
    public void runOpMode() {
        LimelightHiveTracker tracker =
                new LimelightHiveTracker(hardwareMap, LIMELIGHT_NAME, FIDUCIAL_PIPELINE);
        AutoAim aim = new AutoAim();

        // Accept our names or goBILDA's, exactly like Simple Drive, so this runs on
        // either robot with no config change.
        frontLeft  = firstOf("front_left",  "left_front_drive");
        frontRight = firstOf("front_right", "right_front_drive");
        backLeft   = firstOf("back_left",   "left_back_drive");
        backRight  = firstOf("back_right",  "right_back_drive");
        fourWheel = frontLeft != null && frontRight != null
                 && backLeft != null && backRight != null;
        if (!fourWheel) {
            frontLeft  = firstOf("left_drive",  "left");
            frontRight = firstOf("right_drive", "right");
            backLeft = null;
            backRight = null;
        }

        if (frontLeft == null || frontRight == null) {
            telemetry.addLine("Could not find the drive motors.");
            telemetry.addLine("Run \"0. Hardware Check\" to see what names exist.");
            telemetry.update();
            waitForStart();
            return;
        }
        if (fourWheel) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        frontRight.setDirection(fourWheel
                ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);

        telemetry.addLine(fourWheel ? "Found 4 motors -- mecanum." : "Found 2 motors -- tank.");
        telemetry.addLine("Point the Limelight at the target's AprilTag.");
        telemetry.addLine("WHEELS OFF THE GROUND for the first aim test.");
        telemetry.addLine("Press START, then press A to aim.");
        telemetry.update();

        waitForStart();
        tracker.start();

        boolean aimOn = false;
        boolean aPrev = false;

        while (opModeIsActive()) {
            if (gamepad1.a && !aPrev) {
                aimOn = !aimOn;
                aim.reset();   // an old error must not bias the first turn after enabling
            }
            aPrev = gamepad1.a;

            Double bearing = bearingToTarget(tracker);
            AutoAim.Aim command = aimOn
                    ? aim.update(bearing, System.currentTimeMillis())
                    : new AutoAim.Aim(deadband(gamepad1.right_stick_x), bearing != null, false,
                                      bearing == null ? Double.NaN : bearing);

            // Manual override: B hands the turn axis back to the driver for a moment.
            double turn = gamepad1.b ? deadband(gamepad1.right_stick_x) : command.turn;

            double forward = deadband(-gamepad1.left_stick_y);
            double strafe  = deadband(gamepad1.left_stick_x);

            applyDrive(forward, strafe, turn);

            // --- telemetry: the numbers you tune against ---
            telemetry.addData("aim", aimOn ? "ON" : "OFF (A to toggle)");
            telemetry.addData("tag", bearing == null
                    ? "none in view" : String.format(Locale.US, "bearing %+.1f deg", bearing));
            telemetry.addData("turn command", "%+.3f", turn);
            telemetry.addData("aim state", !aimOn ? "--"
                    : !command.hasTarget ? "no target"
                    : command.settled ? "SETTLED"
                    : Math.abs(bearing) <= AutoAim.DEADBAND_DEG ? "centering..." : "turning");
            telemetry.addLine();
            telemetry.addLine("A aim on/off   B hold = turn by hand   left stick drives");
            telemetry.addLine("If it turns AWAY from the tag, flip AutoAim.TURN_SIGN.");
            telemetry.update();
        }

        stopDrive();
        tracker.close();
    }

    /** Bearing to the pinned tag, or the nearest one, or null when none is in view. */
    private static Double bearingToTarget(LimelightHiveTracker tracker) {
        LimelightHiveTracker.Tag tag = (TARGET_TAG_ID >= 0)
                ? tracker.tagWithId(TARGET_TAG_ID)
                : tracker.nearestTag();
        return tag == null ? null : tag.txDeg;
    }

    /**
     * Apply forward/slide from the driver and turn from the aim controller, using
     * the same normalization the drivebases use so a full diagonal plus a turn does
     * not clip into a curve.
     */
    private void applyDrive(double forward, double strafe, double turn) {
        if (fourWheel) {
            double biggest = Math.max(
                    Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
            frontLeft .setPower(DRIVE_SPEED * (forward + strafe + turn) / biggest);
            frontRight.setPower(DRIVE_SPEED * (forward - strafe - turn) / biggest);
            backLeft  .setPower(DRIVE_SPEED * (forward - strafe + turn) / biggest);
            backRight .setPower(DRIVE_SPEED * (forward + strafe - turn) / biggest);
        } else {
            double biggest = Math.max(Math.abs(forward) + Math.abs(turn), 1.0);
            frontLeft .setPower(DRIVE_SPEED * (forward + turn) / biggest);
            frontRight.setPower(DRIVE_SPEED * (forward - turn) / biggest);
        }
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }

    private DcMotor firstOf(String... names) {
        for (String n : names) {
            DcMotor m = hardwareMap.tryGet(DcMotor.class, n);
            if (m != null) return m;
        }
        return null;
    }

    private static double deadband(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : value;
    }
}
