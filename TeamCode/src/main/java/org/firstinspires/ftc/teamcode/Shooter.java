package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Locale;

/**
 * The launcher (flywheel) that shoots POLLEN at the HIVE, plus the windmill that feeds
 * balls into it and the safety gate that blocks the feed when we are not shooting.
 *
 * Robot Configuration names expected on the Control Hub:
 *
 *     launcher        windmill        shooter_gate
 *
 * These match goBILDA's own BIOBUZZ StarterBot example code, so one Robot Configuration
 * drives both their sample and this code. The kit's launcher is a 5203 Yellow Jacket
 * with an encoder, so -- unlike the rest of the robot -- this mechanism is CLOSED LOOP.
 * The windmill is a continuous-rotation servo; the gate is a positional servo.
 *
 * THE GATE. Open = 1.0, closed = 0.5. 0.5 is the correct closed position on this robot,
 * so the open direction is the OTHER side of it, toward 1.0. It is closed at init and
 * opens only for a FORWARD feed; zero or reverse leaves it closed. Every OpMode feeds
 * through setFeed(), so no caller can forget it. An auto that PULSES the windmill uses
 * {@link #holdGateOpen(boolean)} so the gate opens once for the whole feed instead of
 * pumping open/closed on every pulse.
 *
 * WHY VELOCITY, NOT POWER.
 *
 * The ball's range depends on how fast the wheel spins, not how much power goes into
 * it. Power is not speed: the same power gives a different speed as the battery drains,
 * so a power-based table drifts over a match. The launcher has an encoder, so we ask for
 * a speed directly (RUN_USING_ENCODER + setVelocity) and the SDK holds it. Velocity is
 * in encoder ticks per second; a 5203 counts 28 ticks per motor revolution, so
 * RPM = ticksPerSecond / 28 * 60.
 *
 * WHY A MEASURED TABLE, NOT A FORMULA. Range is still not a clean function of wheel
 * speed -- the wheel slips against the ball, the ball launches at an angle, drag acts,
 * and the target sits above the muzzle. So the mapping is a TABLE of measured points
 * and we interpolate; only the y axis is now velocity rather than power.
 * ShooterCalibrateOpMode produces the points. Beyond the last point the table clamps
 * rather than extrapolating, because that is where a shot becomes a guess.
 *
 * FAIL CLOSED. Until MIN_POINTS are recorded rangeToVelocity() returns null and the
 * OpModes refuse to fire. A missing launcher, windmill or gate is harmless -- tryGet
 * returns null and the relevant methods no-op.
 */
public class Shooter {

    public static final String LAUNCHER = "launcher";
    public static final String WINDMILL = "windmill";
    public static final String GATE     = "shooter_gate";

    /** Flip if the wheel spins the wrong way for the ball to leave forward. */
    private static final boolean LAUNCHER_REVERSE = false;
    /** Flip if the windmill pushes balls away from the wheel instead of into it. */
    private static final boolean WINDMILL_REVERSE = true;   // goBILDA's kit reverses it

    /**
     * Gate endpoints, measured on this robot: CLOSED is 0.5. The open direction is the
     * other side of 0.5, so OPEN is 1.0. FTC servo positions are clamped to 0.0..1.0;
     * a negative position is not usable. If it does not quite block when closed, raise
     * the closed value; if it does not quite clear when open, adjust the open value.
     */
    public static final double GATE_OPEN_POSITION   = 1.0;
    public static final double GATE_CLOSED_POSITION = 0.5;

    /**
     * PIDF for the velocity loop. These are goBILDA's values from their BIOBUZZ
     * StarterBot example, tuned for a launcher wheel; P is the term that matters and
     * 12.5 is feed-forward.
     */
    private static final double PIDF_P = 40.0;
    private static final double PIDF_I = 0.0;
    private static final double PIDF_D = 0.0;
    private static final double PIDF_F = 12.5;

    /**
     * Distance -> launcher speed, nearest first. Each pair is {distance in inches,
     * launcher velocity in ticks per second}.
     *
     * FLAT 1200 SINCE 2026-11-18 -- MEASURED STRAIGHT-ON, NOT A FULL CURVE.
     * The flywheel was replaced with a heavier one, so the old rows (1350 at 48 in and
     * 1450 at 76 in, measured 2026-11-02) no longer apply. 1200 was measured
     * straight-on with the new wheel -- fairly consistent, but not perfect -- and is
     * used at every distance until a second distance is measured.
     *
     * Two rows are deliberate even though the speed is the same: rangeToVelocity()
     * refuses to answer with fewer than MIN_POINTS rows. A flat table just means
     * "use the same speed everywhere"; add a second distance once one is measured.
     *
     * RECORD THE BATTERY VOLTAGE with each point ("5. Shooter Calibrate" prints it as a
     * comment on every row). The wheel is closed loop, but it can only hold a speed
     * while the battery has voltage headroom: tuned on a weak battery the wheel could
     * not reach the asked speed, so those numbers really meant a slower wheel and will
     * OVERSHOOT on a fresh pack. If a row does not say its voltage, it could be measured
     * under either condition -- treat it as unknown and re-measure charged.
     *
     * Do not go below MIN_LAUNCH_VELOCITY: below that the wheel can never measure as
     * spun up, so the windmill never opens and there are no shots at all.
     */
    public static final double[][] RANGE_VELOCITY_TABLE = {
            // { inches, ticks per second }
            { 48.0, 1200.0 },   // straight-on shot, measured with the heavier flywheel
            { 76.0, 1200.0 },   // not measured; same speed at the far distance for now
    };

    /** Fewer measured points than this and rangeToVelocity() refuses to answer. */
    public static final int MIN_POINTS = 2;

    /** Ticks per second the launcher must exceed for a ball to leave cleanly. */
    public static final double MIN_LAUNCH_VELOCITY = 800.0;

    private final DcMotorEx launcher;
    private final CRServo windmill;
    private final Servo gate;

    /** True after the gate has been commanded open; avoids re-sending it every loop. */
    private boolean gateOpen = false;

    /**
     * True while a caller has asked the gate to stay open whatever setFeed() is doing.
     * See {@link #holdGateOpen(boolean)}.
     */
    private boolean gateLatched = false;

    /** Whether the last setFeed() asked for a FORWARD feed. Drives the gate when unlatched. */
    private boolean feedForward = false;

    /** Battery voltage sensor, if this hub has one. Only used to record calibration conditions. */
    private final VoltageSensor battery;

    // The velocity we last asked for, so isSpunUp() can be a real measurement rather
    // than a timer: has the wheel actually reached (most of) the target?
    private double commandedVelocity = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        // The launcher is a DcMotorEx because we need setVelocity/getVelocity, which
        // plain DcMotor does not have. tryGet returns null if it is missing or is
        // configured as a plain motor.
        launcher = hardwareMap.tryGet(DcMotorEx.class, LAUNCHER);
        windmill = hardwareMap.tryGet(CRServo.class, WINDMILL);
        gate     = hardwareMap.tryGet(Servo.class, GATE);

        // The Control Hub reports battery voltage; a phone + expansion hub may list more
        // than one. Any of them is fine -- it is only recorded with calibration points.
        VoltageSensor foundBattery = null;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            foundBattery = sensor;
            break;
        }
        battery = foundBattery;

        if (launcher != null) {
            launcher.setDirection(LAUNCHER_REVERSE
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            // FLOAT: a launcher that brakes hard between shots fights itself and cannot
            // be re-triggered quickly. Coast down instead.
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F));
            launcher.setVelocity(0);
        }
        if (windmill != null) {
            windmill.setDirection(WINDMILL_REVERSE
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            windmill.setPower(0);
        }
        if (gate != null) {
            // Closed before START: no ball can reach the wheel until we ask to feed.
            gate.setPosition(GATE_CLOSED_POSITION);
            gateOpen = false;
        }
    }

    public boolean hasLauncher() { return launcher != null; }
    public boolean hasWindmill() { return windmill != null; }
    public boolean hasGate()     { return gate != null; }
    public boolean isGateOpen()  { return gateOpen; }

    /** True when the wheel is spinning fast enough that a ball would leave cleanly. */
    public boolean isSpunUp() {
        if (launcher == null) return false;
        if (commandedVelocity <= 0.0) return false;
        return getVelocity() >= Math.max(MIN_LAUNCH_VELOCITY, commandedVelocity * 0.9);
    }

    /** The wheel's current measured speed, ticks per second. 0 when absent. */
    public double getVelocity() { return launcher == null ? 0.0 : launcher.getVelocity(); }

    /** What we last asked the wheel to hold, ticks per second. */
    public double getCommandedVelocity() { return commandedVelocity; }

    /**
     * The power the motor controller is actually applying to the launcher, -1..1.
     *
     * This is the controller's OUTPUT, not the number we asked for -- with a velocity
     * loop it moves on its own to hold the speed. Useful when the wheel will not reach
     * the asked speed: a power pinned near 1.0 means it is already giving everything the
     * battery has, so the problem is voltage/load, not the request.
     */
    public double getLauncherPower() { return launcher == null ? 0.0 : launcher.getPower(); }

    /**
     * Battery voltage in volts, or NaN when the hub reports none.
     *
     * This does NOT control anything. It is recorded with each calibration point so the
     * table always says what charge it was measured at. A charged battery can spin the
     * wheel faster for the same request, so a table tuned while the battery was weak
     * will overshoot when it is full. Shots only come up short once the voltage sags so
     * far that the wheel can no longer reach the asked speed -- watch actual vs asked.
     */
    public double getBatteryVoltage() {
        return battery == null ? Double.NaN : battery.getVoltage();
    }

    /**
     * Hold a velocity (ticks per second). During a match prefer rangeToVelocity.
     * A NEGATIVE velocity reverses the wheel; that is how the "reverse the shooter"
     * control ejects a jammed ball. It is not clamped to zero, because zeroing a
     * reverse request would make that control silently do nothing.
     */
    public void setVelocity(double ticksPerSecond) {
        commandedVelocity = ticksPerSecond;
        if (launcher != null) launcher.setVelocity(commandedVelocity);
    }

    public void stopLauncher() { setVelocity(0); }

    /**
     * Run the windmill to push a ball into the wheel. +1 feed, -1 reverse, 0 stop.
     *
     * The gate follows the FORWARD feed only: it opens when a shot is being fed and
     * returns to the closed limit for zero or reverse. This is the single choke point
     * every OpMode uses, so the gate cannot be left open after the windmill stops.
     */
    public void setFeed(double power) {
        double p = clamp(power);
        if (windmill != null) windmill.setPower(p);
        feedForward = p > 0.0;
        applyGate();
    }

    /**
     * Hold the gate OPEN (or let go of it again) independently of setFeed().
     *
     * THE GATE IS A SESSION, NOT A PULSE. An auto that stutters the windmill -- on, off,
     * on, off -- used to slam the gate servo through its full travel on every pulse,
     * four or five times in a second: audible pumping, a lot of unnecessary servo work,
     * and no benefit, because once we have decided this robot is shooting, the gate
     * should simply be out of the way until the feed phase ends.
     *
     * While held, setFeed() still runs the windmill (forward, reverse, stop) but leaves
     * the gate alone. The gate opens on the first call, stays open through the pulses
     * and the gaps between them, and goes back to following setFeed() when the hold is
     * released -- so with the feed stopped it closes. Releasing is what puts the safety
     * back: after this, setFeed() controls the gate again as usual.
     *
     * @param hold true = open now and stay open; false = back to following setFeed().
     */
    public void holdGateOpen(boolean hold) {
        gateLatched = hold;
        applyGate();
    }

    /** Open while either the hold is on or the current feed is forward. */
    private void applyGate() {
        setGate(gateLatched || feedForward);
    }

    private void setGate(boolean open) {
        if (gate == null || gateOpen == open) return;
        gate.setPosition(open ? GATE_OPEN_POSITION : GATE_CLOSED_POSITION);
        gateOpen = open;
    }

    /** Everything off. Call when the OpMode ends; this also releases the hold and closes the gate. */
    public void stop() {
        gateLatched = false;
        stopLauncher();
        setFeed(0);
    }

    /**
     * The launcher velocity this distance needs, by linear interpolation of the
     * measured table. Beyond the table it clamps to the nearest endpoint rather than
     * extrapolating.
     *
     * @return ticks per second, or null when the table is too small to trust.
     */
    public Double rangeToVelocity(double distanceIn) {
        if (RANGE_VELOCITY_TABLE.length < MIN_POINTS) return null;

        if (distanceIn <= RANGE_VELOCITY_TABLE[0][0]) return RANGE_VELOCITY_TABLE[0][1];
        int last = RANGE_VELOCITY_TABLE.length - 1;
        if (distanceIn >= RANGE_VELOCITY_TABLE[last][0]) return RANGE_VELOCITY_TABLE[last][1];

        for (int i = 0; i < last; i++) {
            double d0 = RANGE_VELOCITY_TABLE[i][0];
            double d1 = RANGE_VELOCITY_TABLE[i + 1][0];
            if (distanceIn >= d0 && distanceIn <= d1) {
                double t = (distanceIn - d0) / (d1 - d0);
                double v0 = RANGE_VELOCITY_TABLE[i][1];
                double v1 = RANGE_VELOCITY_TABLE[i + 1][1];
                return v0 + t * (v1 - v0);
            }
        }
        return RANGE_VELOCITY_TABLE[last][1];
    }

    /** Human-readable copy of the table, for paste-back into code. */
    public static String tableAsJava() {
        StringBuilder sb = new StringBuilder();
        sb.append("public static final double[][] RANGE_VELOCITY_TABLE = {\n");
        for (double[] row : RANGE_VELOCITY_TABLE) {
            sb.append(String.format(Locale.US, "        { %.1f, %.0f },\n", row[0], row[1]));
        }
        sb.append("};\n");
        return sb.toString();
    }

    private static double clamp(double v) {
        if (v > 1.0) return 1.0;
        if (v < -1.0) return -1.0;
        return v;
    }
}
