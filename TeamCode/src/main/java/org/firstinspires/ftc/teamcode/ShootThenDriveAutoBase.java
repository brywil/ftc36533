package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Locale;

/**
 * Shoot from where the robot already is, then drive: forward 1 square, left 90,
 * forward 1 square, right 90, forward 2.1 squares, left 90, forward 1 square, then
 * back up 0.1 squares.
 *
 * THIS FILE IS NOT AN OPMODE. It holds the whole routine; {@link ShootThenDriveAutoBlue}
 * and {@link ShootThenDriveAutoRed} are the OpModes you pick on the Driver Station, and
 * all each one decides is which alliance it is. Red and blue therefore run the SAME
 * code -- there is nothing to keep in sync -- and alliance() is where an alliance-specific
 * feature (auto-aim at our own goal, for one) plugs in later.
 *
 * WHY RED RUNS THE SAME TURNS AS BLUE. The BIOBUZZ field is not a mirror image left to
 * right where this path ends up. The LOADING ZONES and GARDENS are placed with 180 degree
 * ROTATION (Competition Manual Figures 9-3 and 9-5): the red LOADING ZONE is tile A5, at
 * the REAR end of the red (left) wall, and the blue LOADING ZONE is tile F2, at the
 * AUDIENCE end of the blue (right) wall. A half turn of the field carries one onto the
 * other, and a half turn does not swap left and right. So red's run is blue's run,
 * unchanged, started from blue's start turned 180 degrees about the field centre. (The
 * HIVES, alliance areas and AprilTags DO mirror left/right, which is why mirroring the
 * turns looked right; a mirrored path ends near A2, the wrong end of the red wall.)
 *
 * WHERE TO START (tiles as in manual Figure 9-5: columns A-F left to right from the
 * audience, rows 1-6 from the audience wall back). Both are legal starts under G304,
 * and both appear as examples in manual Figure 11-1.
 *
 *   BLUE: tile D6, against the REAR wall, facing the audience (toward the blue hive) --
 *         the blue circle in Figure 11-1, where this run was tested.
 *         fwd 1 -> D5; left -> facing the right wall; fwd 1 -> E5; right -> facing the
 *         audience; fwd 2.1 -> E2.1; left; fwd 1, back 0.1 -> on the right wall in the
 *         blue LOADING ZONE (F2).
 *
 *   RED:  tile C1, against the AUDIENCE wall, facing the rear (toward the red hive) --
 *         blue's start turned 180 degrees; the red square at C1 in Figure 11-1.
 *         fwd 1 -> C2; left -> facing the left wall; fwd 1 -> B2; right -> facing the
 *         rear; fwd 2.1 -> B4.1; left; fwd 1, back 0.1 -> on the left wall in the red
 *         LOADING ZONE (A5).
 *
 * (The mirrored red path from C6 would end on the left wall around A2.0 -- at the red
 * side's FLOWER, three tiles from the red LOADING ZONE.)
 *
 * NO AIMING. This OpMode assumes the robot is already lined up on the goal when START
 * is pressed. It does not use the Limelight at all -- no tag search, no bearing, no
 * steering -- so it cannot be thrown off by which tags happen to be in view. The
 * launcher just spins up to a fixed speed and feeds.
 *
 * SEQUENCE, on START:
 *   1. ask the launcher for LAUNCH_VELOCITY immediately;
 *   2. wait for the encoder to REPORT the wheel actually at speed, capped so a broken
 *      wheel cannot hang the OpMode, then hold that speed for SETTLE_AFTER_SPINUP_MS;
 *   3. feed with the intake running: the first three pulses are short stutters with a
 *      recovery wait between them, then the windmill runs continuously for the rest of
 *      FEED_MAX_MS -- the stutters keep the first shots consistent, the continuous run
 *      clears the rest of the hopper. After the FIRST pulse, one backwards pulse of
 *      the same length runs the windmill the other way to pull the next ball out of
 *      the feed throat; the normal gap and pulse train then continue unchanged;
 *   4. drive at 50% power: forward 1, left 90, forward 1, right 90, forward 2.1,
 *      left 90, forward 1, then reverse 0.1;
 *   5. stop and report.
 *
 * The drive is plain dead reckoning with {@link ForwardAndTurnAuto}'s measured
 * geometry, so the tile and turn calibration lives in one place. Forward uses
 * ForwardAndTurnAuto.FORWARD_SIGN, the direction that was found to be "forward" on
 * this robot in OpMode 8.
 *
 * WHAT LEFT/RIGHT MEAN HERE. The drivebase's +turn is clockwise, so a right turn is
 * positive and a left turn is negative: see TURN_RIGHT_SIGN / TURN_LEFT_SIGN.
 *
 * SAFETY: wheels off the ground for the first run. The launcher spins on START -- keep
 * clear of the muzzle. Every move has a timeout and stops if a drive encoder is dead.
 */
public abstract class ShootThenDriveAutoBase extends LinearOpMode {

    /**
     * Which alliance this OpMode is for. Answered by the two subclasses. The routine does
     * not use it yet beyond telemetry -- the path is the same for both (see the class
     * comment) -- but it is the one place an alliance-specific feature should read from.
     */
    protected abstract HiveGeometry.Alliance alliance();

    // --- Shooting -----------------------------------------------------------

    /** Launcher wheel speed for the shot, ticks/s. Same as the other autos. */
    public static final double LAUNCH_VELOCITY = 1300.0;

    /**
     * Cap on waiting for the wheel to report at-speed before feeding anyway. Raised
     * from 3 s because the wheel was being fed before it finished spinning up (shots
     * came out short). If the telemetry still says "launcher at speed: NO" at 5 s, the
     * wheel genuinely cannot reach LAUNCH_VELOCITY -- lower that value, charge the
     * battery, or check the launcher's encoder cable.
     */
    public static final long SPIN_UP_MAX_WAIT_MS = 5000;

    /**
     * Hold the wheel at speed for this long after it reports spun up, before any ball is
     * fed. The velocity loop can still be ringing when it first crosses the target, so
     * this lets it settle -- and gives every run the same starting point.
     */
    public static final long SETTLE_AFTER_SPINUP_MS = 2000;

    /**
     * How long the whole feed phase may run before the robot moves on. The first
     * {@link #FEED_SMALL_PULSE_COUNT} pulses are short stutters with recovery between
     * them; after that the windmill runs continuously for the rest of this time. 12 s
     * leaves room for the drive inside the 30 s AUTO -- raise it only if the drive is
     * comfortably faster than that.
     */
    public static final long FEED_MAX_MS = 12000;

    /** How many short starter pulses before the windmill goes continuous. */
    public static final int FEED_SMALL_PULSE_COUNT = 3;

    /** On-time of each starter pulse. Kept short so the first balls feed gently. */
    public static final long FEED_SMALL_PULSE_ON_MS = 120;

    /** Minimum gap with the windmill off between starter pulses. */
    public static final long FEED_SMALL_PULSE_OFF_MS = 200;

    /**
     * Windmill power for the backwards pulse that follows the FIRST forward pulse.
     * The first pulse pushes a ball up against the wheel; running the windmill
     * backwards pulls the next ball back out of the feed throat, so it is not
     * double-fed the instant feeding resumes.
     *
     * The reverse lasts exactly as long as a forward pulse -- {@link
     * #FEED_SMALL_PULSE_ON_MS} -- and is then followed by the normal gap and pulse
     * train, so none of the existing timings change. Pulses 2 and later keep the
     * original stop-then-pulse pattern. -1.0 is full reverse, the same as the
     * driver's jam-clearing button.
     */
    public static final double FEED_REVERSE_AFTER_FIRST_PULSE = -1.0;

    /**
     * After a pulse, wait until the wheel is back to this fraction of the speed we asked
     * for before feeding again. 0.95 leaves room for the velocity loop's small steady
     * error, so normal running is not treated as "not recovered".
     */
    public static final double FEED_RECOVER_FRACTION = 0.95;

    /** Cap on the recovery wait, so a weak battery cannot stall the auto in the feed phase. */
    public static final long FEED_RECOVER_MAX_MS = 1500;

    /**
     * Run the intake while feeding, at this power. The windmill pushes balls in from
     * the hopper; running the intake at the same time keeps pulling from the field and
     * can dislodge a ball sitting on the intake. goBILDA's own launch routine adds
     * intake power for exactly this reason.
     */
    public static final double FEED_INTAKE_POWER = 1.0;

    // --- Post-shot movement -------------------------------------------------

    /** The four forward legs, in squares, in drive order. */
    public static final double FORWARD_1_SQUARES = 1.0;
    public static final double FORWARD_2_SQUARES = 1.0;
    public static final double FORWARD_3_SQUARES = 2.1;
    public static final double FORWARD_4_SQUARES = 1.0;

    /** Reverse this many squares at the very end. */
    public static final double BACKWARD_FINAL_SQUARES = 0.1;

    /** How far each turn goes, in degrees. */
    public static final double TURN_DEG = 90.0;

    /** +1 is clockwise, -1 counter-clockwise. Turn order is left, right, left. */
    public static final double TURN_RIGHT_SIGN = 1.0;
    public static final double TURN_LEFT_SIGN = -1.0;

    /** Movement power, 50% as requested, for both driving and turning. */
    public static final double MOVEMENT_POWER = 0.5;

    public static final int TOLERANCE_TICKS = 15;
    public static final long FORWARD_TIMEOUT_MS = 8000;
    public static final long TURN_TIMEOUT_MS = 5000;
    public static final long ENCODER_STARTUP_GRACE_MS = 500;

    private static final String REACHED = "reached";

    private TankDrivebase drive;
    private Intake intake;
    private ShooterControls shooter;
    private double ticksPerInch;

    /** CSV of this run, for checking power/rate afterwards. Null-safe when it cannot open. */
    private TeamRunLog log;

    @Override
    public void runOpMode() {
        drive = new TankDrivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new ShooterControls(new Shooter(hardwareMap));

        ticksPerInch = ForwardAndTurnAuto.TICKS_PER_WHEEL_REV
                / (Math.PI * ForwardAndTurnAuto.WHEEL_DIAMETER_IN);

        // A file, not just the screen: the power/rate can be checked after the run.
        log = TeamRunLog.open(hardwareMap.appContext,
                "shoot-then-drive-" + (alliance() == HiveGeometry.Alliance.RED ? "red" : "blue"));
        log.row("elapsed_ms", "event", "detail", "commanded", "actual", "power",
                "left_ticks", "right_ticks", "left_rate", "right_rate");
        log.row("", "config", "alliance=" + alliance()
                        + " launch=" + num(LAUNCH_VELOCITY)
                        + " move_power=" + num(MOVEMENT_POWER)
                        + " battery=" + num(shooter.shooter().getBatteryVoltage()),
                "", "", "", "", "", "", "");

        telemetry.addData("alliance", alliance());
        telemetry.addLine("Auto: shoot (no aim), then fwd 1, left 90, fwd 1, right 90, fwd 2.1, left 90, fwd 1.");
        if (alliance() == HiveGeometry.Alliance.RED) {
            telemetry.addLine("RED START: tile C1, against the AUDIENCE wall, facing the red hive.");
        } else {
            telemetry.addLine("BLUE START: tile D6, against the REAR wall, facing the blue hive.");
        }
        telemetry.addLine("Robot must ALREADY be lined up. WHEELS OFF THE GROUND first run.");
        telemetry.addLine();
        telemetry.addData("launcher", shooter.shooter().hasLauncher() ? "OK" : "MISSING");
        telemetry.addData("windmill", shooter.shooter().hasWindmill() ? "OK" : "MISSING");
        telemetry.addData("movement power", "%.0f%%", MOVEMENT_POWER * 100);
        telemetry.addData("log", log.isOpen() ? log.path() : "OFF -- could not open");
        telemetry.addLine("Pull with: adb pull " + TeamRunLog.folderOnHub() + " .");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // --- 1-3: spin up, wait for at-speed, feed ---
        shooter.setManual(true);
        shooter.setArmed(true);
        // Pin this auto's own speed. Without this it would inherit the teleop's
        // DEFAULT_MANUAL_VELOCITY, so lowering the teleop base would change the auto too.
        shooter.setManualVelocity(LAUNCH_VELOCITY);
        shooter.update(null);            // manual: command the manual speed now

        long spinBegin = System.currentTimeMillis();
        boolean ready = false;
        while (opModeIsActive()) {
            ready = shooter.isSpunUp();
            long waited = System.currentTimeMillis() - spinBegin;
            telemetry.addData("spinning up launcher",
                    "asked %.0f  actual %.0f ticks/s  power %+.2f  (%d ms)",
                    shooter.getCommandedVelocity(), shooter.getVelocity(),
                    shooter.shooter().getLauncherPower(), waited);
            telemetry.update();
            log.row(waited, "spinup", ready ? "ready" : "",
                    num(shooter.getCommandedVelocity()), num(shooter.getVelocity()),
                    num(shooter.shooter().getLauncherPower()), "", "", "", "");
            if (ready || waited >= SPIN_UP_MAX_WAIT_MS) break;
            idle();
        }

        telemetry.addData("launcher at speed", ready
                ? "yes" : "NO -- timed out, check battery / encoder cable");
        telemetry.update();

        // Hold the wheel at speed before the first ball. The velocity loop can still be
        // ringing as it first crosses the target; feeding immediately can give a different
        // first shot on every run. Keep the launcher commanded the whole time.
        long settleBegin = System.currentTimeMillis();
        while (opModeIsActive()
                && System.currentTimeMillis() - settleBegin < SETTLE_AFTER_SPINUP_MS) {
            long waited = System.currentTimeMillis() - settleBegin;
            telemetry.addData("settling before feed", "%d / %d ms",
                    waited, SETTLE_AFTER_SPINUP_MS);
            telemetry.addData("  launcher", "asked %.0f  actual %.0f ticks/s  power %+.2f",
                    shooter.getCommandedVelocity(), shooter.getVelocity(),
                    shooter.shooter().getLauncherPower());
            telemetry.update();
            log.row(waited, "settle", "", num(shooter.getCommandedVelocity()),
                    num(shooter.getVelocity()),
                    num(shooter.shooter().getLauncherPower()), "", "", "", "");
            idle();
        }

        // The first few balls are short stutters: pulse, stop, let the wheel recover, pulse
        // again. That keeps those shots at the same speed instead of shoving a ball in
        // while the wheel is still down. After the starter pulses the windmill runs
        // continuously for the rest of the phase to clear the hopper.
        intake.setPower(FEED_INTAKE_POWER);
        long feedBegin = System.currentTimeMillis();
        long phaseBegin = feedBegin;
        int pulsesDone = 0;
        boolean pulsing = true;
        boolean reversing = false;   // the one backwards pulse after pulse 1
        boolean continuous = false;
        shooter.shooter().setFeed(1.0);

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();
            long fed = now - feedBegin;
            if (fed >= FEED_MAX_MS) break;

            long phase = now - phaseBegin;
            double asked = shooter.getCommandedVelocity();
            double actual = shooter.getVelocity();
            boolean recovered = asked > 0 && actual >= asked * FEED_RECOVER_FRACTION;

            if (!continuous) {
                if (pulsing) {
                    if (phase >= FEED_SMALL_PULSE_ON_MS) {
                        pulsing = false;
                        phaseBegin = now;
                        pulsesDone++;
                        if (pulsesDone == 1) {
                            // Backwards for exactly one forward-pulse width, then the
                            // normal gap below runs untouched.
                            reversing = true;
                            shooter.shooter().setFeed(FEED_REVERSE_AFTER_FIRST_PULSE);
                        } else {
                            shooter.shooter().setFeed(0.0);
                        }
                    }
                } else if (reversing) {
                    if (phase >= FEED_SMALL_PULSE_ON_MS) {
                        reversing = false;
                        phaseBegin = now;
                        shooter.shooter().setFeed(0.0);
                    }
                } else if (phase >= FEED_SMALL_PULSE_OFF_MS
                        && (recovered || phase >= FEED_RECOVER_MAX_MS)) {
                    if (pulsesDone >= FEED_SMALL_PULSE_COUNT) {
                        continuous = true;
                        shooter.shooter().setFeed(1.0);
                    } else {
                        pulsing = true;
                        phaseBegin = now;
                        shooter.shooter().setFeed(1.0);
                    }
                }
            }

            String state = continuous ? "CONTINUOUS"
                    : pulsing ? "stutter " + pulsesDone + "/" + FEED_SMALL_PULSE_COUNT
                    : reversing ? "reversing after pulse 1"
                    : (recovered ? "recovered" : "recovering");
            telemetry.addData("feed phase", "%d / %d ms  %s  (intake on)",
                    fed, FEED_MAX_MS, state);
            telemetry.addData("  launcher", "asked %.0f  actual %.0f ticks/s  power %+.2f  %s",
                    asked, actual, shooter.shooter().getLauncherPower(),
                    shooter.isSpunUp() ? "READY" : "SLOW");
            if (asked > 0 && actual < asked * 0.90
                    && shooter.shooter().getLauncherPower() > 0.95) {
                telemetry.addLine("LAUNCHER SATURATED -- battery cannot hold the speed");
            }
            telemetry.update();
            String phaseName = continuous ? "feed-cont"
                    : pulsing ? "feed-on"
                    : reversing ? "feed-rev" : "feed-off";
            log.row(fed, phaseName, state,
                    num(asked), num(actual),
                    num(shooter.shooter().getLauncherPower()), "", "", "", "");
            idle();
        }
        shooter.shooter().setFeed(0.0);
        intake.stop();
        shooter.stop();
        sleep(300);                       // let the wheel coast off before driving

        // --- 4: post-shot movement at 50% ---
        String fwd1 = "", turnL = "", fwd2 = "", turnR1 = "", fwd3 = "", turnL2 = "", fwd4 = "", lastBack = "";
        if (opModeIsActive()) {
            fwd1 = driveForward("after: forward 1", FORWARD_1_SQUARES);
            turnL = turn("after: turn left 90", TURN_LEFT_SIGN);
            fwd2 = driveForward("after: forward 1", FORWARD_2_SQUARES);
            turnR1 = turn("after: turn right 90", TURN_RIGHT_SIGN);
            fwd3 = driveForward("after: forward 2.1", FORWARD_3_SQUARES);
            turnL2 = turn("after: turn left 90", TURN_LEFT_SIGN);
            fwd4 = driveForward("after: forward 1", FORWARD_4_SQUARES);
            lastBack = driveBackward("after: backward 0.1", BACKWARD_FINAL_SQUARES);
        }

        drive.stop();
        shooter.stop();

        telemetry.addData("launch", ready ? "fired" : "wheel never reached speed");
        telemetry.addData("forward 1", fwd1);
        telemetry.addData("turn left 90", turnL);
        telemetry.addData("forward 1", fwd2);
        telemetry.addData("turn right 90", turnR1);
        telemetry.addData("forward 2.1", fwd3);
        telemetry.addData("turn left 90", turnL2);
        telemetry.addData("forward 1", fwd4);
        telemetry.addData("backward 0.1", lastBack);
        telemetry.update();

        // Keep the row-by-row data on the Hub after the OpMode ends.
        log.row("", "result", "launch=" + (ready ? "fired" : "no-speed"), "", "", "", "", "", "", "");
        log.row("", "result", "forward 1=" + fwd1, "", "", "", "", "", "", "");
        log.row("", "result", "turn left 90=" + turnL, "", "", "", "", "", "", "");
        log.row("", "result", "forward 1=" + fwd2, "", "", "", "", "", "", "");
        log.row("", "result", "turn right 90=" + turnR1, "", "", "", "", "", "", "");
        log.row("", "result", "forward 2.1=" + fwd3, "", "", "", "", "", "", "");
        log.row("", "result", "turn left 90=" + turnL2, "", "", "", "", "", "", "");
        log.row("", "result", "forward 1=" + fwd4, "", "", "", "", "", "", "");
        log.row("", "result", "backward 0.1=" + lastBack, "", "", "", "", "", "", "");
        log.row("", "file", log.path() == null ? "no file" : log.path(),
                "", "", "", "", "", "", "");
        log.close();

        sleep(2000);
    }

    private int turnTicks() {
        return (int) Math.round(
                (TURN_DEG / 360.0) * Math.PI
                * ForwardAndTurnAuto.TRACK_WIDTH_IN * ticksPerInch);
    }

    /** Ticks to drive physically forward by this many squares (FORWARD_SIGN applied). */
    private int forwardTicks(double squares) {
        return (int) Math.round(ForwardAndTurnAuto.FORWARD_SIGN * squares
                * ForwardAndTurnAuto.TILE_IN * ticksPerInch);
    }

    /** Drive physically forward this many squares at MOVEMENT_POWER. */
    private String driveForward(String label, double squares) {
        int ticks = forwardTicks(squares);
        return runMove(label + " (" + format(squares) + " sq)", ticks, ticks,
                MOVEMENT_POWER, FORWARD_TIMEOUT_MS);
    }

    /** Drive physically backward this many squares: the negative of forward. */
    private String driveBackward(String label, double squares) {
        int ticks = -forwardTicks(squares);
        return runMove(label + " (" + format(squares) + " sq)", ticks, ticks,
                MOVEMENT_POWER, FORWARD_TIMEOUT_MS);
    }

    /** Spin in place by TURN_DEG in the given direction (+1 clockwise, -1 CCW). */
    private String turn(String label, double sign) {
        int ticks = (int) (sign * turnTicks());
        return runMove(label, ticks, -ticks, MOVEMENT_POWER, TURN_TIMEOUT_MS);
    }

    /** Same per-side encoder-checked move as the other autos. */
    private String runMove(String label, int leftDelta, int rightDelta,
                           double power, long timeoutMs) {
        int leftStart = drive.leftTicks();
        int rightStart = drive.rightTicks();

        drive.driveToTicks(leftStart + leftDelta, rightStart + rightDelta, power);

        long begin = System.currentTimeMillis();
        while (opModeIsActive()) {
            int leftMoved = drive.leftTicks() - leftStart;
            int rightMoved = drive.rightTicks() - rightStart;
            long elapsed = System.currentTimeMillis() - begin;

            boolean leftDead = leftDelta != 0 && leftMoved == 0;
            boolean rightDead = rightDelta != 0 && rightMoved == 0;
            boolean encodersDead = elapsed > ENCODER_STARTUP_GRACE_MS
                    && (leftDead || rightDead);

            double leftRate = elapsed > 0 ? leftMoved * 1000.0 / elapsed : 0.0;
            double rightRate = elapsed > 0 ? rightMoved * 1000.0 / elapsed : 0.0;

            telemetry.addData("move", label);
            telemetry.addData("  left", "%d / %d ticks", leftMoved, leftDelta);
            telemetry.addData("  right", "%d / %d ticks", rightMoved, rightDelta);
            telemetry.addData("  power", "%.0f%%  rate L %.0f / R %.0f ticks/s",
                    power * 100.0, leftRate, rightRate);
            telemetry.update();
            log.row(elapsed, "move", label, "", "", num(power),
                    leftMoved, rightMoved, num(leftRate), num(rightRate));

            if (encodersDead) {
                drive.stop();
                sleep(150);
                return "STOPPED: drive encoder not reading -- check the cables";
            }

            boolean arrived = Math.abs(leftDelta - leftMoved) <= TOLERANCE_TICKS
                    && Math.abs(rightDelta - rightMoved) <= TOLERANCE_TICKS;
            if (arrived) {
                drive.stop();
                sleep(150);
                return REACHED;
            }
            if (elapsed > timeoutMs) {
                drive.stop();
                sleep(150);
                return "TIMED OUT -- check geometry";
            }
            idle();
        }

        drive.stop();
        return "stopped";
    }

    /** Trim a trailing ".0" so "3.0" reads as "3". */
    private static String format(double value) {
        return (value == Math.floor(value))
                ? String.valueOf((long) value)
                : String.valueOf(value);
    }

    /** One number for a CSV cell: fixed decimal point, never a locale comma. */
    private static String num(double value) {
        return String.format(Locale.US, "%.2f", value);
    }
}
