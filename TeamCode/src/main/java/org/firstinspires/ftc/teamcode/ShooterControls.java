package org.firstinspires.ftc.teamcode;

/**
 * The shooter's state machine, separate from any one OpMode's button layout.
 *
 * Both drive TeleOps need to shoot, and {@link ShooterOpMode} does too. Copying the
 * arm / manual-vs-auto / spin-up-gate / feed logic into each would be three places to
 * fix the same bug -- the same reason the HIVE gate lives in {@link HiveGate}. So the
 * logic is here once, and each OpMode only decides which buttons call these methods.
 *
 * WHAT THIS OWNS
 *   - whether the launcher is armed
 *   - AUTO (velocity from the measured table and the tag range) vs MANUAL (fixed velocity)
 *   - the spin-up guard: a ball is never fed into a wheel that has not reached speed
 *   - the feed request, gated on that guard
 *
 * WHAT IT DOES NOT OWN
 *   - reading the gamepad (the OpMode decides which button means what)
 *   - the camera (the OpMode passes in a range, or null)
 *   - the table itself (that is {@link Shooter})
 *
 * THE SPIN-UP GUARD IS A MEASUREMENT, NOT A TIMER. The launcher has an encoder, so
 * {@link Shooter#isSpunUp()} asks the wheel how fast it is actually going. That is
 * strictly better than the open-loop timer this class used to run: it does not care
 * how the wheel was loaded or what the battery is doing.
 *
 * CAMERA-OPTIONAL. If there is no camera, AUTO can never answer, so MANUAL is the only
 * useful mode; {@link #fallBackToManual()} selects it. A caller that knows the camera
 * is missing should call that once at init.
 */
public class ShooterControls {

    /** Starting manual velocity, and the step the tune buttons move it by. */
    public static final double DEFAULT_MANUAL_VELOCITY = 1250.0;   // goBILDA's launch value
    public static final double MANUAL_STEP = 25.0;

    private final Shooter shooter;

    private boolean armed = false;
    private boolean manual = false;
    private double manualVelocity = DEFAULT_MANUAL_VELOCITY;

    /** The range last seen by update(), for telemetry. NaN when there was none. */
    private double lastRangeIn = Double.NaN;

    /** Feed power sent last, -1..1. */
    private double feedPower = 0.0;

    public ShooterControls(Shooter shooter) {
        this.shooter = shooter;
    }

    public Shooter shooter() { return shooter; }

    // --- arm ----------------------------------------------------------------

    public void toggleArm() { armed = !armed; if (!armed) shooter.stopLauncher(); }
    public void setArmed(boolean on) { armed = on; if (!armed) shooter.stopLauncher(); }
    public boolean isArmed() { return armed; }

    // --- AUTO / MANUAL ------------------------------------------------------

    public void toggleManual() { manual = !manual; }
    public void setManual(boolean on) { manual = on; }
    public boolean isManual() { return manual; }

    /** MANUAL is the only mode that can work with no camera; select it. */
    public void fallBackToManual() { manual = true; }

    public double getManualVelocity() { return manualVelocity; }

    public void nudgeManualVelocity(double delta) {
        manualVelocity = Math.max(0.0, manualVelocity + delta);
    }

    // --- per-loop update ----------------------------------------------------

    /**
     * Compute and apply the launcher velocity for this loop.
     *
     * @param rangeIn distance to the target tag in inches, or null when there is no
     *                usable reading (no camera, no tag, no 3D pose).
     */
    public void update(Double rangeIn) {
        lastRangeIn = (rangeIn == null) ? Double.NaN : rangeIn;

        double targetVelocity;
        if (!armed) {
            targetVelocity = 0.0;
        } else if (manual) {
            targetVelocity = manualVelocity;
        } else if (rangeIn != null) {
            Double fromTable = shooter.rangeToVelocity(rangeIn);
            // null means the table has too few measured points -- do not guess.
            targetVelocity = (fromTable == null) ? 0.0 : fromTable;
        } else {
            targetVelocity = 0.0;   // AUTO with no range: refuse rather than guess
        }

        shooter.setVelocity(targetVelocity);
    }

    /** The wheel's measured speed, ticks per second. */
    public double getVelocity() { return shooter.getVelocity(); }

    /** What the wheel was asked to hold, ticks per second. */
    public double getCommandedVelocity() { return shooter.getCommandedVelocity(); }

    /** True when the wheel is actually at speed (a measurement, not a timer). */
    public boolean isSpunUp() { return shooter.isSpunUp(); }

    public double getLastRangeIn() { return lastRangeIn; }

    // --- feed ---------------------------------------------------------------

    /**
     * Request a feed. Forward is gated on the measured spin-up, so a cold wheel never
     * gets a ball; reverse (clearing a jam) always works.
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

    /** "OFF", "MANUAL", "TABLE", "NO RANGE", or "NO TABLE" -- why the speed is what it is. */
    public String velocitySource() {
        if (!armed) return "OFF";
        if (manual) return "MANUAL";
        if (Double.isNaN(lastRangeIn)) return "NO RANGE";
        return shooter.rangeToVelocity(lastRangeIn) == null ? "NO TABLE" : "TABLE";
    }

    /** "RUNNING", "REVERSE", "ready", or "held". */
    public String feedState() {
        if (feedPower > 0) return "RUNNING";
        if (feedPower < 0) return "REVERSE";
        return isSpunUp() ? "ready" : "held";
    }
}
