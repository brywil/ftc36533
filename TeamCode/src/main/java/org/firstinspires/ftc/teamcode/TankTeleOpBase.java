package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Locale;

/**
 * Tank TeleOp for the goBILDA BIOBUZZ StarterBot / kitbot -- one motor per side.
 *
 * THIS FILE IS NOT AN OPMODE. It holds the logic; {@link TankTeleOpRed} and
 * {@link TankTeleOpBlue} are the OpModes you pick on the Driver Station, and the only
 * thing they decide is which tag cluster is our goal.
 *
 * ---------------------------------------------------------------------------
 * WHY TWO OPMODES AND NOT A SELECTOR BUTTON
 *
 * To aim, the robot must know which goal is ITS goal, and that fact cannot be worked
 * out from the camera. Facing the opponent's goal, "aim at the tag I can see" lines
 * the robot up on THEIR goal and fires. The alliance has to be TOLD, never inferred.
 *
 * Picking "2a. Tank TeleOp (RED)" tells it. That is a choice the driver already makes
 * every match, it is visible on the Driver Station to anyone watching, and it cannot
 * be silently wrong the way a forgotten pre-match toggle can.
 *
 * The two subclasses exist instead of two copies of this file so that a fix here
 * cannot be made in one alliance's TeleOp and forgotten in the other's.
 * ---------------------------------------------------------------------------
 *
 * ONE STICK DRIVES. A skid-steer cannot strafe, so instead of "left stick forward,
 * right stick turn", the LEFT stick does everything: up/down is forward/back, and
 * left/right turns. The right stick also turns, so a driver can steer with whichever
 * thumb is free.
 *
 *   left stick up/down    forward / back
 *   left stick left/right turn
 *   right stick left/right turn (same thing; either thumb works)
 *   right trigger         precision creep -- scales the DRIVER's inputs down for
 *                         lining up (it deliberately does not scale the aim's turn)
 *   RIGHT BUMPER          aims at our goal, runs the intake, AND fires -- see below
 *   left bumper           intake (roller + servos) forward only, no aim, no fire
 *   square                intake ROLLER reverse + windmill reverse; servos stay still
 *                         (also cancels aim and fire while held)
 *   dpad up / down        launcher speed -/+  (launcher runs in MANUAL, always on)
 *
 * THE LAUNCHER IS ALWAYS RUNNING. There is no arm button: from START it spins at the
 * manual speed (ShooterControls.DEFAULT_MANUAL_VELOCITY), so the driver only tunes it
 * with the dpad. Because it is manual, the launcher does not need the Limelight -- but
 * the AIM does, so this OpMode uses the camera only for the turn axis.
 *
 * RIGHT BUMPER DOES THREE THINGS: it runs the intake, turns the robot toward our goal
 * tag, and feeds. Aiming and firing at the same time is the point -- hold RB as you
 * approach and the robot lines itself up while the balls go.
 *
 * Feeding is still gated on the launcher's spin-up measurement, so a cold wheel never
 * gets a ball. It is deliberately NOT gated on being aimed -- see REQUIRE_AIM_TO_FIRE
 * for why that gate was written, why it was turned off, and when to turn it back on.
 *
 * WINDMILL WITH THE INTAKE. The windmill is the feed servo that pushes balls into the
 * launcher. It runs forward with the right bumper, but only once the launcher wheel is
 * measured at speed AND the robot has been aimed at least once during this press, so a
 * press during spin-up or during the turn is held, not lost. It runs in reverse with
 * square (clears a jam, always allowed). The left bumper runs the intake WITHOUT aim
 * and without the windmill, so a driver can load without firing.
 */
public abstract class TankTeleOpBase extends LinearOpMode {

    /** Gamepad sticks rarely read exactly zero when you let go. Below this is "not touched". */
    private static final double DEADBAND = 0.05;

    /** Multiplier at full precision trigger. */
    private static final double CREEP_SCALE = 0.30;

    /** Top speed as a fraction of full power. 1.0 is full; lower it to slow the robot. */
    private static final double DRIVE_GAIN = 1.0;

    /** Manual launcher speed step for the dpad up/down buttons. */
    private static final double MANUAL_STEP = ShooterControls.MANUAL_STEP;

    /** Must match the Robot Configuration name of the Limelight. */
    private static final String LIMELIGHT_NAME = "limelight";

    /** The fiducial (AprilTag) pipeline you configured on the camera. */
    private static final int FIDUCIAL_PIPELINE = 1;

    /**
     * How many recent updates the cluster bearing is averaged over (a rolling window).
     * At this loop's rate that spans a few camera frames.
     *
     * Averaging the members WITHIN one frame (see bearingToScoringCluster) cannot remove
     * everything: which members the camera reports still changes frame to frame -- now
     * two, now three -- so the frame's group mean still steps between values, and a turn
     * command built from it steps with it. ForwardOneAimAndShootAutoBase hit the same thing
     * and measured the jump at 0..7 deg; the window there is what stops the robot
     * steering at whichever subset happened to be detected that frame. This is that
     * window, for the same reason.
     */
    private static final int AIM_SMOOTH_FRAMES = 5;

    /**
     * Drop the window after this long with none of the cluster in view, so an angle from
     * before the goal left the frame cannot bias the bearing re-acquired after it.
     */
    private static final long AIM_SMOOTH_RESET_MS = 400;

    /**
     * OFF. RB aims, runs the intake, and fires -- and the fire is gated by the
     * launcher's spin-up measurement only, exactly as it was before aim existed.
     *
     * WHY THIS IS OFF. It was on first, on the theory that a ball fed mid-turn leaves
     * sideways. In practice it made the button dead: "aimed" requires the bearing to
     * sit inside AutoAim.DEADBAND_DEG for SETTLE_MS, that did not happen on the kitbot,
     * and so the windmill never spun forward at all. A dead button is worse than an
     * unaimed shot, and worse again because a driver cannot tell it apart from a broken
     * robot. The driver's press should always do something.
     *
     * TURN IT ON ONLY ONCE "AIMED" IS PROVEN. Run "7. Auto Aim (April Tag)" and watch
     * the aim line: if it reaches AIMED reliably, aim-gated firing is worth having and
     * this flag is a one-line change. Until then the latch below is unused code.
     *
     * When on, the result is LATCHED for the rest of the press, so a single frame where
     * the tag is lost or the bearing wobbles out of the deadband cannot stutter the feed
     * the way the spin-up gate used to. Ignored entirely when there is no camera.
     */
    private static final boolean REQUIRE_AIM_TO_FIRE = false;

    /**
     * Which cluster of BIOBUZZ tags is THIS robot's goal. Answered by the subclasses,
     * and the only thing they have to answer: see HiveGeometry.Cell for the clusters.
     * RED_FAR is the "RED SCORING" cluster (ids 30-33); BLUE_FAR is "BLUE SCORING"
     * (ids 42-45). The AUDIENCE clusters are not goals and are never aimed at.
     */
    protected abstract HiveGeometry.Cell scoringCell();

    @Override
    public void runOpMode() {
        TankDrivebase drive = new TankDrivebase(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        ShooterControls shooter = new ShooterControls(new Shooter(hardwareMap));
        LimelightHiveTracker tracker =
                new LimelightHiveTracker(hardwareMap, LIMELIGHT_NAME, FIDUCIAL_PIPELINE);
        AutoAim aim = new AutoAim();

        // The camera is optional: a robot without one still has to drive. RB then just
        // runs the intake and fires, exactly as it did before aim existed.
        final boolean aimAvailable = tracker.hasLimelight();

        // The launcher is always on and always manual -- no camera needed for it.
        shooter.setManual(true);
        shooter.setArmed(true);

        boolean upPrev = false, downPrev = false;
        boolean aimHeldPrev = false;
        boolean aimedThisHold = false;

        telemetry.addLine("Ready. Point the robot downfield before START.");
        telemetry.addData("alliance", "%s -> aim at %s (tags %s)",
                scoringCell().alliance, scoringCell().clusterName, idList(scoringCell()));
        telemetry.addData("limelight", aimAvailable ? "OK" : "MISSING -- aim disabled");
        telemetry.addData("launcher", shooter.shooter().hasLauncher() ? "OK" : "MISSING");
        telemetry.addData("windmill", shooter.shooter().hasWindmill() ? "OK" : "MISSING");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        tracker.start();

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            // --- aim: the right bumper takes over the TURN axis while it is held ---
            boolean aimHeld = aimAvailable && gamepad1.right_bumper && !gamepad1.square;
            if (aimHeld && !aimHeldPrev) {
                // Rising edge: an old error must not bias the first turn of a new press,
                // and the fire gate re-arms for this press.
                aim.reset();
                clearAimSmoothing();   // same reason: no angle from before this press
                aimedThisHold = false;
            }
            aimHeldPrev = aimHeld;

            Double bearing = aimHeld ? bearingToScoringCluster(tracker, now) : null;
            AutoAim.Aim aimCommand = aimHeld
                    ? aim.update(bearing, now)
                    : new AutoAim.Aim(0.0, false, false, false, Double.NaN);
            if (aimHeld && aimCommand.settled) aimedThisHold = true;

            // --- drive: arcade, no strafe (a skid-steer cannot slide) ---
            double forward = square(deadband(-gamepad1.left_stick_y));
            double stickTurn = clamp(square(deadband(gamepad1.left_stick_x))
                                   + square(deadband(gamepad1.right_stick_x)), -1.0, 1.0);

            // The driver keeps the last word on turning: move either stick and the aim
            // lets go for as long as it is moved. A held RB must never be a robot that
            // will not let you steer away from something.
            boolean driverTurning = stickTurn != 0.0;
            boolean aimDriving = aimHeld && !driverTurning;

            double scale = DRIVE_GAIN * (1.0 - (1.0 - CREEP_SCALE) * gamepad1.right_trigger);
            // Creep trims the DRIVER's inputs only. Scaling the aim's turn by it would
            // leave the correction too weak to close the last few degrees, and the robot
            // would sit at "centering..." that never gets there.
            double forwardCommand = forward * scale;
            double turnCommand = aimDriving ? aimCommand.turn : stickTurn * scale;

            drive.driveRobotCentric(forwardCommand, turnCommand);

            // --- intake, from the bumpers and square ---
            // The windmill is handled further down, through the gates -- so it is
            // deliberately NOT in this chain, even though RB also runs the intake.
            double intakeMotor = 0.0, intakeServos = 0.0;
            if (gamepad1.square) {
                intakeMotor = -1.0;   // reverse the roller to unjam / outake
                // servos stay 0: they would fight the roller if reversed
            } else if (gamepad1.right_bumper) {
                intakeMotor = 1.0;
                intakeServos = 1.0;
            } else if (gamepad1.left_bumper) {
                intakeMotor = 1.0;
                intakeServos = 1.0;
                // no aim, no windmill: load without firing
            }
            intake.setMotor(intakeMotor);
            intake.setServos(intakeServos);

            // --- feed, through the spin-up gate and the aim gate ---
            // Forward only once the wheel is measured at speed AND (when we can aim, and
            // there is something to aim at) the robot has been aimed during this press.
            // Reverse (square) is always allowed, for jam-clearing.
            //
            // The "something to aim at" clause is what keeps RB from going dead. With no
            // goal tag in view the aim can never settle, so without it the windmill never
            // spins forward and the fire button does nothing at all -- which is a worse
            // failure than an unaimed shot, because the driver cannot tell it apart from
            // a broken robot.
            boolean goalInView = bearing != null;
            boolean aimGateOpen = !REQUIRE_AIM_TO_FIRE || !aimAvailable || aimedThisHold
                               || !goalInView;
            shooter.requestFeed(gamepad1.right_bumper && !gamepad1.square && aimGateOpen,
                                gamepad1.square);

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
            telemetry.addData("drive", "fwd %+.2f  turn %+.2f%s",
                    forwardCommand, turnCommand, aimDriving ? "  (aim)" : "");
            telemetry.addData("intake", "motor %+.0f  servos %+.0f",
                    intakeMotor, intakeServos);
            telemetry.addData("launcher", "asked %.0f  actual %.0f ticks/s  %s",
                    shooter.getCommandedVelocity(), shooter.getVelocity(),
                    shooter.isSpunUp() ? "READY" : "spinning up");
            telemetry.addData("feed", shooter.feedState());
            telemetry.addData("aim", aimText(aimAvailable, aimHeld, bearing, aimCommand));
            telemetry.addData("aim cluster", "%d of %d tags   frame %s   smoothed %s",
                    lastClusterMembers, HiveGeometry.CLUSTER_MEMBER_COUNT,
                    deg(lastFrameBearingDeg), deg(bearing));
            telemetry.addData("fire gate", !REQUIRE_AIM_TO_FIRE ? "not gated on aim"
                    : !aimAvailable ? "open (no camera)"
                    : aimedThisHold ? "open (aimed this press)"
                    : !goalInView ? "open (no goal tag in view -- unaimed shot)"
                    : "waiting for aim");
            telemetry.addLine();
            telemetry.addLine("Left stick drives.  Hold RB = aim + intake + fire.");
            telemetry.addLine("LB intake only   square reverse   dpad up/down = speed");
            telemetry.update();
        }

        drive.stop();
        intake.stop();
        shooter.stop();
        tracker.close();
    }

    /**
     * The aim telemetry line: one answer, six cases. Pulled out of the loop because a
     * nested ternary six deep is how a telemetry bug hides.
     *
     * The coasting case is checked BEFORE "no tag", because both of them have a null
     * bearing -- the difference is that one of them still has the wheels turning, and
     * that is the thing worth seeing when the motion stutters.
     */
    private String aimText(boolean aimAvailable, boolean aimHeld, Double bearing,
                           AutoAim.Aim command) {
        if (!aimAvailable) return "no camera";
        if (!aimHeld) return "off (hold RB)";
        if (command.coasting) return "coasting (tag dropped a frame)";
        if (bearing == null) return "NO TAG -- looking for " + scoringCell().clusterName;
        return command.settled ? "AIMED"
                : String.format(Locale.US, "turning %+.1f deg", bearing);
    }

    /**
     * Bearing to the CENTRE of our SCORING cluster, in degrees, or null when none of it
     * is in view.
     *
     * Deliberately not nearestTag(). The nearest tag of any kind can be an AUDIENCE
     * cluster or, worse, the opponent's goal -- and aiming at either lines the robot up
     * to score for the other alliance. Only our own SCORING cluster is a valid answer.
     *
     * AND DELIBERATELY NOT THE NEAREST MEMBER OF THAT CLUSTER EITHER, which is what this
     * did first. A CELL is four tags across one sticker, so they span real angle: the
     * left member and the right member of the same cluster read several degrees apart.
     * Picking the nearest member per frame means the target SWITCHES as the robot turns
     * -- aim at the left tag, then the middle one is nearer, then the left again -- and
     * each switch throws the bearing sideways in one step. The robot hunts back and
     * forth between members and never settles, which is exactly what the driver saw:
     * "twitching and undecided which one".
     *
     * Averaging the members present gives one continuous number that means "the middle
     * of the goal", which is the thing worth aiming at anyway, and it cancels per-tag
     * solve noise besides -- the same reasoning LimelightHiveTracker.gate() uses when it
     * averages a cluster rather than trusting whichever member was detected first.
     *
     * AND NOT JUST THIS FRAME'S MEAN EITHER. Which members the camera reports changes
     * frame to frame, so a mean over one frame still steps: two members give one answer,
     * three give another, and the turn is built from whichever subset was seen. So the
     * per-frame mean is passed through a short rolling window (AIM_SMOOTH_FRAMES) -- the
     * same fix ForwardOneAimAndShootAutoBase applies for the same reason -- and the input
     * becomes one continuous number instead of a sequence of jumps.
     *
     * @param now the loop's clock, so a long gap can be timed without a second read.
     */
    private Double bearingToScoringCluster(LimelightHiveTracker tracker, long now) {
        HiveGeometry.Cell want = scoringCell();
        double sum = 0.0;
        int found = 0;
        for (LimelightHiveTracker.Tag t : tracker.recognize()) {
            if (t.cell != want) continue;
            sum += t.txDeg;
            found++;
        }
        lastClusterMembers = found;

        if (found == 0) {
            // None of the goal in view this frame. Report null and let AutoAim's own
            // COAST_MS decide how long a gap may last -- but keep the window, so the next
            // frame that DOES see the goal is still smoothed against the recent past.
            // Only a gap that drags on drops it, because an angle from before the goal
            // left the frame is worse than no smoothing at all.
            lastFrameBearingDeg = null;
            if (now - lastClusterFrameMs > AIM_SMOOTH_RESET_MS) clearAimSmoothing();
            return null;
        }

        // This frame's own mean first, kept for telemetry, then the windowed answer.
        // While the window is still filling this is a mean of what exists so far, so the
        // very first frame after aim starts is that frame's own value -- no lag, and no
        // old data to bias it.
        lastFrameBearingDeg = sum / found;
        bearingWin[bearingNext] = lastFrameBearingDeg;
        bearingNext = (bearingNext + 1) % AIM_SMOOTH_FRAMES;
        if (bearingCount < AIM_SMOOTH_FRAMES) bearingCount++;
        lastClusterFrameMs = now;

        double windowed = 0.0;
        for (int i = 0; i < bearingCount; i++) windowed += bearingWin[i];
        return windowed / bearingCount;
    }

    /**
     * How many members of our cluster the last bearing was averaged from, for telemetry.
     * Zero means none were in view. Worth showing: it says whether "aimed" was decided
     * from the whole goal or from a single tag that happened to be detected.
     */
    private int lastClusterMembers = 0;

    /**
     * This frame's raw cluster mean, before smoothing -- the value that used to step
     * straight into AutoAim. Null when the last frame saw none of the cluster. Shown
     * only so the window's work is visible: if this jumps and the smoothed value does
     * not, the window is doing its job.
     */
    private Double lastFrameBearingDeg = null;

    /** The rolling window of recent cluster means, and how much of it is filled. */
    private final double[] bearingWin = new double[AIM_SMOOTH_FRAMES];
    private int bearingCount = 0, bearingNext = 0;

    /** When the cluster was last actually seen, so a long gap can drop the window. */
    private long lastClusterFrameMs = 0L;

    /**
     * Forget the rolling window. Called when aim starts, for the same reason AutoAim is
     * reset then: an angle from before this press must not bias the first turn of it.
     */
    private void clearAimSmoothing() {
        bearingCount = 0;
        bearingNext = 0;
        lastFrameBearingDeg = null;
    }

    /** "30, 31, 32, 33" -- for telemetry, so the pinned cluster is never a mystery. */
    private static String idList(HiveGeometry.Cell cell) {
        int[] ids = cell.memberIds();
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < ids.length; i++) {
            if (i > 0) sb.append(", ");
            sb.append(ids[i]);
        }
        return sb.toString();
    }

    /** "+1.2" / "-3.4", or "--" for a bearing that is missing. Telemetry only. */
    private static String deg(Double v) {
        return (v == null || v.isNaN()) ? "--" : String.format(Locale.US, "%+.1f", v);
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
