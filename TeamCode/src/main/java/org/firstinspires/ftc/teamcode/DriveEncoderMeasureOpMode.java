package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Measures the drive encoder's counts per wheel revolution, for
 * {@link ForwardAndTurnAuto#TICKS_PER_WHEEL_REV}.
 *
 * WHY THIS EXISTS. That constant is the one Autonomous number a ruler cannot give
 * you -- it comes off the motor's gearbox, and the code's placeholder (537.7) may be
 * the wrong gearbox entirely. The other two geometry numbers you measure with a
 * tape: WHEEL_DIAMETER_IN is the wheel across its tread, and TRACK_WIDTH_IN is the
 * centre of the left wheel to the centre of the right.
 *
 * HOW TO USE IT
 *
 *  1. WHEELS OFF THE GROUND. Put the robot on blocks -- the wheels must turn freely.
 *  2. Run this OpMode, then press START.
 *  3. Press Y to zero the counters.
 *  4. Turn ONE wheel by hand through exactly one full revolution. Put a mark on the
 *     tyre and a mark on the robot so you can see when it comes back around.
 *  5. Read "left ticks" and "right ticks". That number is your counts per wheel rev.
 *
 * Hold A (or B for reverse) to turn the wheels with the motor instead of by hand.
 * The count is the same either way; turning by hand is simply easier to stop at
 * exactly one revolution.
 *
 * WHAT THE READING MEANS
 *
 *  - A count that stays at 0 while the wheel turns means that side's encoder cable
 *    is missing or in the wrong port -- the exact problem Autonomous item #1 warns
 *    about. Fix the cable before trusting Autonomous.
 *  - If you already know the motor (a goBILDA 5203 counts 28 per MOTOR turn), then
 *    counts-per-wheel-rev / 28 is the gearbox ratio. 537.7 / 28 = 19.2, so 537.7 is
 *    the 19.2:1 gearbox -- not the 5.23:1 its comment claims. Measure and use what
 *    the robot actually has.
 */
@TeleOp(name = "9. Drive Encoder Measure", group = "Bringup")
public class DriveEncoderMeasureOpMode extends LinearOpMode {

    /** Slow, so you can watch the wheel and stop it at one revolution. */
    private static final double SPIN_POWER = 0.15;

    private DcMotor left, right;

    @Override
    public void runOpMode() {
        left  = hardwareMap.tryGet(DcMotor.class, TankDrivebase.LEFT_MOTOR);
        right = hardwareMap.tryGet(DcMotor.class, TankDrivebase.RIGHT_MOTOR);

        if (left == null || right == null) {
            telemetry.addLine("Could not find the drive motors.");
            telemetry.addLine("Run \"0. Hardware Check\" to see what names exist.");
            telemetry.addLine("Expected: \"" + TankDrivebase.LEFT_MOTOR + "\" and \""
                    + TankDrivebase.RIGHT_MOTOR + "\".");
            telemetry.update();
            waitForStart();
            return;
        }

        zeroEncoders();

        telemetry.addLine("WHEELS OFF THE GROUND. Turn one wheel by hand one full turn,");
        telemetry.addLine("then read the ticks. That is TICKS_PER_WHEEL_REV.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean yPrev = false;

        while (opModeIsActive()) {
            // Y zeroes both counters, so "one revolution" is simply what is on screen.
            if (gamepad1.y && !yPrev) zeroEncoders();
            yPrev = gamepad1.y;

            // Hold A to drive both wheels forward, B for reverse. By hand is fine too.
            double power = gamepad1.a ? SPIN_POWER : gamepad1.b ? -SPIN_POWER : 0.0;
            left.setPower(power);
            right.setPower(power);

            int lt = left.getCurrentPosition();
            int rt = right.getCurrentPosition();

            telemetry.addData("left ticks", lt);
            telemetry.addData("right ticks", rt);
            telemetry.addLine();
            telemetry.addLine("Y = zero the counters");
            telemetry.addLine("Hold A = spin forward, B = spin reverse (or turn by hand)");
            telemetry.addLine();
            telemetry.addLine("A side stuck at 0 while its wheel turns = encoder cable");
            telemetry.addLine("missing or in the wrong port.");
            telemetry.addLine();
            telemetry.addLine("Paste the reading into ForwardAndTurnAuto.TICKS_PER_WHEEL_REV.");
            telemetry.update();
        }

        left.setPower(0);
        right.setPower(0);
    }

    /** Reset both encoders and leave them readable, but not driving closed-loop. */
    private void zeroEncoders() {
        for (DcMotor m : new DcMotor[] { left, right }) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
