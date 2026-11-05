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

    /**
     * Starting manual velocity, and the step the tune buttons move it by. 1300 is what
     * "2. Tank TeleOp" was tuned to on 2026-10-15 -- shots landed there and the wheel
     * measured at speed -- so the driver starts where it was tuned rather than 50 low.
     */
    public static final double DEFAULT_MANUAL_VELOCITY = 1300.0;
    public static final double MANUAL_STEP = 25.0;

    private final Shooter shooter;

    private boolean armed = false;
    private boolean manual = false;
    private double manualVelocity = DEFAULT_MANUAL_VELOCITY;

    /**
     * Trim added to the table's AUTO speed, ticks/s. Lets the driver walk the speed up
     * or down WITHOUT leaving AUTO, so it works while a tag is in view -- which is
     * exactly when the table is in charge and the plain manual dpad was being ignored.
     * The effective speed (table + trim) is what telemetry shows, so it can be written
     * back into the table as a measured point.
     */
    private double autoTrimVelocity = 0.0;

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

    /** Set the MANUAL speed directly -- e.g. Autonomous pinning a known-good value. */
    public void setManualVelocity(double velocity) { manualVelocity = Math.max(0.0, velocity); }

    public void nudgeManualVelocity(double delta) {
        manualVelocity = Math.max(0.0, manualVelocity + delta);
    }

    /**
     * Trim the AUTO (table) speed by delta ticks/s, without leaving AUTO. This is how
     * the dpad keeps working while the Limelight has a tag: the table still sets the
     * range-based speed, and this corrects it from what the driver sees.
     */
    public void nudgeAutoTrim(double delta) { autoTrimVelocity += delta; }

    /** Trim currently applied to the table speed, ticks/s. */
    public double getAutoTrimVelocity() { return autoTrimVelocity; }

    /** Drop the trim, e.g. when starting a new calibration or after pasting it in. */
    public void clearAutoTrim() { autoTrimVelocity = 0.0; }

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
            // null means the table has too few measured points -- do not guess. The trim
            // cannot invent a speed, so it only applies when the table answered.
            targetVelocity = (fromTable == null)
                    ? 0.0 : Math.max(0.0, fromTable + autoTrimVelocity);
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
     * Request a feed. Forward only STARTS on a wheel that is measured up to speed, so
     * a cold wheel never gets a ball; reverse (clearing a jam) always works.
     *
     * ONCE STARTED, KEEP GOING -- until the wheel collapses below MIN_LAUNCH_VELOCITY.
     * Pushing a ball in always slows the wheel for a moment, and shutting the feed on
     * that dip made the windmill stutter: ball in, wheel drops under the spin-up line,
     * feed shuts, wheel speeds back up, feed opens, next ball -- a cycle about once a
     * second, which looks like the windmill twitching and the wheel changing speed.
     * The power already sent (feedPower) is what tells us a feed is under way.
     */
    public void requestFeed(boolean feed, boolean reverse) {
        double p = 0.0;
        if (reverse) {
            p = -1.0;   // reverse wins over a feed: that is what a jam deserves
        } else if (feed) {
            boolean running = feedPower > 0;
            boolean fastEnough = running
                    ? shooter.getVelocity() >= Shooter.MIN_LAUNCH_VELOCITY
                    : isSpunUp();
            p = fastEnough ? 1.0 : 0.0;
        }
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
