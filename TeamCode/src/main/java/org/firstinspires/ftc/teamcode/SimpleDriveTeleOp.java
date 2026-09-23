package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * The smallest possible "push the stick, robot moves" OpMode. Start here.
 *
 * It deliberately does almost nothing: no IMU, no field-centric mode, no mode
 * switching. Every one of those is a thing that can be missing or misconfigured,
 * and on the first day you want exactly one thing to prove -- that a joystick can
 * make wheels turn.
 *
 * It works on whatever it finds:
 *   - four motors -> full mecanum driving, including sideways
 *   - two motors  -> ordinary tank/arcade driving
 *   - anything missing -> it says so on the Driver Station instead of crashing
 *
 * It also accepts two different sets of motor names, ours and the ones goBILDA's
 * sample code uses, so following either set of instructions works.
 *
 * Controls:
 *   left stick        drive (and slide sideways, if you have mecanum wheels)
 *   right stick L/R   turn
 *
 * Once this works, move on to "2. Mecanum TeleOp" for the real driving code.
 */
@TeleOp(name = "1. Simple Drive", group = "Bringup")
public class SimpleDriveTeleOp extends LinearOpMode {

    /** Top speed, as a fraction. Kept low on purpose -- fast robots break things. */
    private static final double SPEED = 0.5;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private boolean fourWheel;

    @Override
    public void runOpMode() {
        // tryGet gives back null when a name is not in the configuration, instead of
        // stopping the OpMode with an error. That lets us explain the problem in
        // words rather than leaving you with a screen of red Java text.
        frontLeft  = firstOf("front_left",  "left_front_drive");
        frontRight = firstOf("front_right", "right_front_drive");
        backLeft   = firstOf("back_left",   "left_back_drive");
        backRight  = firstOf("back_right",  "right_back_drive");

        fourWheel = frontLeft != null && frontRight != null
                 && backLeft != null && backRight != null;

        if (!fourWheel) {
            // Fall back to a two-motor robot.
            frontLeft  = firstOf("left_drive",  "left");
            frontRight = firstOf("right_drive", "right");
            backLeft = null;
            backRight = null;
        }

        if (frontLeft == null || frontRight == null) {
            telemetry.addLine("Could not find the drive motors.");
            telemetry.addLine();
            telemetry.addLine("This code looks for either:");
            telemetry.addLine("  four motors: front_left, front_right,");
            telemetry.addLine("               back_left, back_right");
            telemetry.addLine("  or two:      left_drive, right_drive");
            telemetry.addLine();
            telemetry.addLine("Run \"0. Hardware Check\" to see what names your");
            telemetry.addLine("configuration actually has.");
            telemetry.update();
            waitForStart();
            return;
        }

        // The left and right motors face opposite directions on the robot, so one
        // side has to be told to run backwards for "forward" to mean forward. WHICH
        // side is reversed differs between the two robots, so this matches each
        // drivebase's own convention rather than assuming one:
        //   four motors -> MecanumDrivebase: left reversed, right forward
        //   two motors  -> TankDrivebase (goBILDA kitbot): left forward, right reversed
        // If a side drives the wrong way, fix it in the drivebase classes so every
        // OpMode stays in agreement -- not here.
        if (fourWheel) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        telemetry.addLine(fourWheel ? "Found 4 motors -- mecanum driving."
                                    : "Found 2 motors -- tank driving.");
        telemetry.addLine();
        telemetry.addLine("WHEELS OFF THE GROUND for the first test.");
        telemetry.addLine("Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // The stick reads negative when pushed forward, which is backwards from
            // what anyone expects, so flip it here once and forget about it.
            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double turn    =  gamepad1.right_stick_x;

            if (fourWheel) {
                // Divide everything by the biggest request so the robot goes where
                // you pointed. Without this, a full diagonal plus a turn asks a motor
                // for more than 100% power, it gives what it has, and the robot
                // curves off somewhere you did not ask for.
                double biggest = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
                frontLeft .setPower(SPEED * (forward + strafe + turn) / biggest);
                frontRight.setPower(SPEED * (forward - strafe - turn) / biggest);
                backLeft  .setPower(SPEED * (forward - strafe + turn) / biggest);
                backRight .setPower(SPEED * (forward + strafe - turn) / biggest);
            } else {
                double biggest = Math.max(Math.abs(forward) + Math.abs(turn), 1.0);
                frontLeft .setPower(SPEED * (forward + turn) / biggest);
                frontRight.setPower(SPEED * (forward - turn) / biggest);
            }

            telemetry.addData("mode", fourWheel ? "4 motors (mecanum)" : "2 motors (tank)");
            telemetry.addData("sticks", "forward %+.2f  sideways %+.2f  turn %+.2f",
                    forward, strafe, turn);
            telemetry.addLine();
            telemetry.addLine("Nothing moving? Check the wheels are not jammed, the");
            telemetry.addLine("battery is in, and that you pressed START (not just INIT).");
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }

    /** First of these names that exists in the configuration, or null. */
    private DcMotor firstOf(String... names) {
        for (String n : names) {
            DcMotor m = hardwareMap.tryGet(DcMotor.class, n);
            if (m != null) return m;
        }
        return null;
    }
}
