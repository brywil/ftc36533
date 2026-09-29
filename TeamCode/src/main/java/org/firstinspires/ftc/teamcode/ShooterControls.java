package org.firstinspires.ftc.teamcode;

/**
 * The shooter's state machine, separate from any one OpMode's button layout.
 *
 * Both drive TeleOps need to shoot, and {@link ShooterOpMode} does too. Copying the
 * arm / manual-vs-auto / spin-up-gate / feed logic into each would be three places
 * to fix the same bug -- the same reason the HIVE gate lives in {@link HiveGate}
 * rather than in every tracker. So the logic is here once, and each OpMode only
 * decides which buttons call these methods.
 *
 * WHAT THIS OWNS
 *   - whether the flywheel is armed
 *   - AUTO (power from the measured table and the tag range) vs MANUAL (fixed power)
 *   - the spin-up guard: a ball is never fed into a wheel that has not reached speed
 *   - the feed request, gated on that guard
 *
 * WHAT IT DOES NOT OWN
 *   - reading the gamepad (the OpMode decides which button means what)
 *   - the camera (the OpMode passes in a range, or null)
 *   - the table itself (that is {@link Shooter})
 *
 * CAMERA-OPTIONAL. If there is no camera, AUTO can never answer, so MANUAL is the
 * only useful mode; {@link #fallBackToManual()} switches to it and says so. A caller
 * that knows the camera is missing should call that once at init.
 */
public class ShooterControls {

    /**
     * How long after the commanded power changes before a ball may be fed. There is
     * no encoder, so the wheel's actual speed cannot be measured; this timer is the
     * open-loop stand-in for "it has spun up". Tune by ear, then by where the first
     * ball of a burst lands.
     */
    public static final long SPINUP_MS = 700;

    /** Power changes smaller than this are drift, not a new shot -- do not re-wait. */
    private static final double POWER_EPSILON = 0.02;

    /** Starting manual power, and the step the power-tune buttons move it by. */
    public static final double DEFAULT_MANUAL_POWER = 0.6;

    private final Shooter shooter;

    private boolean armed = false;
    private boolean manual = false;
    private double manualPower = DEFAULT_MANUAL_POWER;

    // The power actually sent to the motor, and when it last changed -- the two
    // things the spin-up guard is computed from.
    private double commandedPower = 0.0;
    private long powerChangedAtMs = 0L;

    /** The range last seen by update(), for telemetry. NaN when there was none. */
    private double lastRangeIn = Double.NaN;

    /** Feed motor power sent last, -1..1. */
    private double feedPower = 0.0;

    public ShooterControls(Shooter shooter) {
        this.shooter = shooter;
    }

    public Shooter shooter() { return shooter; }

    // --- flywheel arm -------------------------------------------------------

    public void toggleArm() { armed = !armed; }
    public void setArmed(boolean on) { armed = on; }
    public boolean isArmed() { return armed; }

    // --- AUTO / MANUAL ------------------------------------------------------

    public void toggleManual() { manual = !manual; }
    public void setManual(boolean on) { manual = on; }
    public boolean isManual() { return manual; }

    /** MANUAL is the only mode that can work with no camera; say so by selecting it. */
    public void fallBackToManual() { manual = true; }

    public double getManualPower() { return manualPower; }

    public void nudgeManualPower(double delta) {
        manualPower = clamp01(manualPower + delta);
    }

    // --- per-loop update ----------------------------------------------------

    /**
     * Compute and apply the flywheel power for this loop.
     *
     * @param rangeIn distance to the target tag in inches, or null when there is no
     *                usable reading (no camera, no tag, no 3D pose).
     * @param nowMs   a monotonic clock, e.g. System.currentTimeMillis().
     */
    public void update(Double rangeIn, long nowMs) {
        lastRangeIn = (rangeIn == null) ? Double.NaN : rangeIn;

        double targetPower;
        if (!armed) {
            targetPower = 0.0;
        } else if (manual) {
            targetPower = manualPower;
        } else if (rangeIn != null) {
            Double fromTable = shooter.rangeToPower(rangeIn);
            // null means the table has too few measured points -- do not guess.
            targetPower = (fromTable == null) ? 0.0 : fromTable;
        } else {
            targetPower = 0.0;   // AUTO with no range: refuse rather than guess
        }

        if (Math.abs(targetPower - commandedPower) > POWER_EPSILON) {
            commandedPower = targetPower;
            powerChangedAtMs = nowMs;
        }
        shooter.setPower(commandedPower);
    }

    /** True when the wheel has been at its commanded power long enough to feed. */
    public boolean isSpunUp() {
        return commandedPower > 0.0
                && (System.currentTimeMillis() - powerChangedAtMs) >= SPINUP_MS;
    }

    public double getCommandedPower() { return commandedPower; }
    public double getLastRangeIn() { return lastRangeIn; }

    // --- feed ---------------------------------------------------------------

    /**
     * Request a feed. The forward direction is gated on spin-up, so a cold wheel
     * never gets a ball; reverse (clearing a jam) always works.
     */
    public void requestFeed(boolean feed, boolean reverse) {
        double p = 0.0;
        if (feed && isSpunUp()) p = 1.0;
        else if (reverse) p = -1.0;
        feedPower = p;
        shooter.setFeed(p);
    }

    public double getFeedPower() { return feedPower; }

    /** Everything off. Call when the OpMode ends. */
    public void stop() { shooter.stop(); }

    // --- telemetry helpers --------------------------------------------------

    /** "OFF", "MANUAL", "TABLE", or "NO TABLE" -- why the power is what it is. */
    public String powerSource() {
        if (!armed) return "OFF";
        if (manual) return "MANUAL";
        if (Double.isNaN(lastRangeIn)) return "NO RANGE";
        return shooter.rangeToPower(lastRangeIn) == null ? "NO TABLE" : "TABLE";
    }

    /** "RUNNING", "REVERSE", "ready", or "held" -- for a single telemetry line. */
    public String feedState() {
        if (feedPower > 0) return "RUNNING";
        if (feedPower < 0) return "REVERSE";
        return isSpunUp() ? "ready" : "held";
    }

    private static double clamp01(double v) {
        if (v > 1.0) return 1.0;
        if (v < 0.0) return 0.0;
        return v;
    }
}
