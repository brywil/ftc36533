package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Shoot from where the robot already is. No driving, no turning, no aiming -- the
 * robot is assumed to be lined up on the goal when START is pressed, so this only
 * spins the launcher up and feeds.
 *
 * WHY THIS EXISTS SEPARATELY: "8. Auto: 2 Squares, Turn 90, Launch" spends most of its
 * time driving and turning. When the robot is already in shooting position (a match
 * where it starts lined up, or a bench test of just the shooter), all that motion is
 * wasted and makes it harder to tell whether the *shot* is good. This OpMode isolates
 * the shooting sequence so it can be tuned and tested on its own.
 *
 * SEQUENCE, on START:
 *   1. ask the launcher for LAUNCH_VELOCITY -- immediately, nothing before it;
 *   2. wait for the encoder to REPORT the wheel actually at speed (isSpunUp is a real
 *      measurement, not a timer), capped so a broken wheel cannot hang the OpMode;
 *   3. run the windmill for FEED_DURATION_MS, pushing balls into the wheel;
 *   4. stop and report.
 *
 * SAFETY: the launcher spins the instant START is pressed. Wheels off the ground and
 * nothing in front of the muzzle for the first run, and keep hands away from the wheel
 * the whole time. If the wheel never reaches speed the telemetry says so and the shot
 * is thrown short -- check the launcher's encoder cable and the battery.
 */
@Autonomous(name = "10. Auto: Launch Only (lined up)", group = "Auto")
public class LaunchOnlyAuto extends LinearOpMode {

    /**
     * Launcher wheel speed for the shot, in encoder ticks per second. Matches
     * ForwardAndTurnAuto's value so the two autos shoot the same: 1300 is a little
     * above goBILDA's 1250 baseline. Tune by watching the ball, not the number.
     */
    public static final double LAUNCH_VELOCITY = 1300.0;

    /**
     * Wait at most this long for the wheel to report at-speed before feeding anyway.
     * If this times out, telemetry says "launcher at speed: NO" -- the wheel could not
     * reach LAUNCH_VELOCITY (weak battery, or a missing encoder cable), and the shot
     * will be short. It is a cap so the OpMode cannot hang; it is not a target.
     */
    public static final long SPIN_UP_MAX_WAIT_MS = 3000;

    /** How long to run the windmill, pushing balls into the wheel. */
    public static final long FEED_DURATION_MS = 5000;

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap);

        telemetry.addLine("Auto: launch only. Robot must ALREADY be lined up on the goal.");
        telemetry.addLine("WHEELS OFF THE GROUND for the first run.");
        telemetry.addData("launcher", shooter.hasLauncher() ? "OK" : "MISSING");
        telemetry.addData("windmill", shooter.hasWindmill() ? "OK" : "MISSING");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Spin up right away -- this is the whole point of this OpMode.
        shooter.setVelocity(LAUNCH_VELOCITY);

        long begin = System.currentTimeMillis();
        boolean ready = false;
        while (opModeIsActive()) {
            ready = shooter.isSpunUp();
            long waited = System.currentTimeMillis() - begin;
            telemetry.addData("spinning up launcher", "%.0f / %.0f ticks/s  (%d ms)",
                    shooter.getVelocity(), shooter.getCommandedVelocity(), waited);
            telemetry.update();
            if (ready || waited >= SPIN_UP_MAX_WAIT_MS) break;
            idle();
        }

        telemetry.addData("launcher at speed", ready
                ? "yes" : "NO -- timed out, check battery / encoder cable");
        telemetry.update();

        // Feed. The windmill pushes whatever is in the hopper into the spinning wheel.
        shooter.setFeed(1.0);
        sleep(FEED_DURATION_MS);
        shooter.setFeed(0.0);

        shooter.stop();

        telemetry.addData("launch", ready ? "fired" : "wheel never reached speed");
        telemetry.addLine();
        telemetry.addLine("Done. If it did not reach speed, check the launcher's encoder");
        telemetry.addLine("cable and the battery -- see WIRING.md.");
        telemetry.update();

        // Leave the result up so it can be read on the Driver Station.
        sleep(2000);
    }
}
