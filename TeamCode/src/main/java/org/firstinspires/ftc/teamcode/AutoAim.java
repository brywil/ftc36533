package org.firstinspires.ftc.teamcode;

/**
 * Turns a tag's bearing into a turn command, so the robot can point itself at a
 * target instead of a driver doing it by eye.
 *
 * The input is the same "tx" the Limelight reports and 4. HIVE Bench prints: the
 * angle between the camera's crosshair and the tag, positive when the tag is to the
 * right. Drive that to zero and the camera -- and so the shooter -- is pointed at
 * the tag.
 *
 * NO HARDWARE HERE. This class takes a bearing and hands back a number. The OpMode
 * decides what to do with it (turn a drivebase, a turret, a servo), which is what
 * keeps the control loop testable and reusable as the robot grows.
 *
 * ---------------------------------------------------------------------------
 * WHY A LOST TARGET IS A SAFETY CASE, NOT A DETAIL
 *
 * When the tag leaves the frame the bearing does not exist, and a controller that
 * keeps turning on the last angle it saw will spin to find a target that moved.
 * Worse, fast turning causes motion blur, which is often what loses the tag in the
 * first place, so turning harder to recover can make it worse. So: no bearing means
 * no turn, full stop. Recovery is the driver's job (or a future search mode), not
 * this class guessing.
 * ---------------------------------------------------------------------------
 *
 * TUNING, in the order to do it, on blocks with the wheels free:
 *
 *  1. Confirm the DIRECTION before anything else. With the tag to the right of the
 *     crosshair, the robot must turn right (clockwise). If it turns away, flip
 *     TURN_SIGN. This cannot be reasoned out from the mounting -- it must be seen.
 *  2. Set KP so the robot turns briskly toward the tag without screaming across.
 *     Raise it until it just starts to overshoot, then back off.
 *  3. If it hunts side to side around center, add KD a little at a time. KD fights
 *     the speed the error is changing at; too much and the robot twitches on vision
 *     noise, so use the least that stops the hunting.
 *  4. Widen DEADBAND_DEG if it never settles. The camera has a few tenths of a
 *     degree of noise, and a deadband smaller than that can never be satisfied.
 */
public class AutoAim {

    /**
     * Which way a positive bearing should turn. +1 means "tag on the right -> turn
     * right". Verify on blocks; flip to -1 if the robot turns away from the tag.
     */
    public static final double TURN_SIGN = 1.0;

    /** Turn power per degree of error. The main tuning knob. */
    public static final double KP = 0.020;

    /** Damping. 0 is pure P; add a little only if it hunts around center. */
    public static final double KD = 0.0;

    /**
     * Turn power ceiling. Kept low on purpose: a vision loop that turns fast blurs
     * the tag out of its own view, so the fast fix is often the one that loses the
     * target. Turn speed is bounded by what the camera can still see.
     */
    public static final double MAX_TURN = 0.35;

    /**
     * Least turn power that still moves the robot. Below this, the wheels stall
     * against friction and nothing happens, so a small error would never be fixed.
     * Keep it small enough that it does not overshoot from the edge of the deadband.
     */
    public static final double MIN_TURN = 0.06;

    /**
     * How close counts as aimed, in degrees. The camera's own angle noise is a few
     * tenths of a degree, so this must be comfortably larger or the robot can never
     * be "settled".
     */
    public static final double DEADBAND_DEG = 1.5;

    /** How long the bearing must stay inside the deadband before "settled" is true. */
    public static final long SETTLE_MS = 250;

    /**
     * A fixed angle offset added to the aim, in degrees. Non-zero if the shooter
     * points somewhere other than straight down the camera, or if shots land
     * consistently to one side. Positive aims right.
     */
    public static final double AIM_OFFSET_DEG = 0.0;

    /** The controller's answer for one iteration. */
    public static final class Aim {
        /** Turn command, -1..1. Positive is clockwise, matching the drivebases. */
        public final double turn;
        /** True when a tag with a usable bearing is in view. */
        public final boolean hasTarget;
        /** True when the bearing has stayed inside the deadband for SETTLE_MS. */
        public final boolean settled;
        /** The bearing used, in degrees; NaN when there was no target. */
        public final double bearingDeg;

        Aim(double turn, boolean hasTarget, boolean settled, double bearingDeg) {
            this.turn = turn;
            this.hasTarget = hasTarget;
            this.settled = settled;
            this.bearingDeg = bearingDeg;
        }
    }

    private boolean haveLast = false;
    private double lastTurnError = 0.0;
    private long lastMs = 0L;
    private long centeredSinceMs = -1L;

    /** Forget all history. Call when enabling aim, so an old error cannot bias the first turn. */
    public void reset() {
        haveLast = false;
        lastTurnError = 0.0;
        lastMs = 0L;
        centeredSinceMs = -1L;
    }

    /**
     * @param bearingDeg the target's bearing, or null when no tag is in view.
     * @param nowMs      a monotonically increasing clock, e.g. SystemClock.uptimeMillis().
     */
    public Aim update(Double bearingDeg, long nowMs) {
        if (bearingDeg == null) {
            // Lost the target: stop, drop history, refuse to claim settled.
            haveLast = false;
            centeredSinceMs = -1L;
            return new Aim(0.0, false, false, Double.NaN);
        }

        double angleError = bearingDeg - AIM_OFFSET_DEG;   // degrees, + is target right
        double turnError  = TURN_SIGN * angleError;        // + means "turn positive"

        if (Math.abs(angleError) <= DEADBAND_DEG) {
            if (centeredSinceMs < 0) centeredSinceMs = nowMs;
            boolean settled = (nowMs - centeredSinceMs) >= SETTLE_MS;
            // Keep the derivative history current so the first frame after leaving
            // the deadband does not see a huge, false rate of change.
            haveLast = true;
            lastTurnError = turnError;
            lastMs = nowMs;
            return new Aim(0.0, true, settled, bearingDeg);
        }
        centeredSinceMs = -1L;

        double raw = KP * turnError;

        if (KD != 0.0 && haveLast && nowMs > lastMs) {
            double dtSec = (nowMs - lastMs) / 1000.0;
            double rate = (turnError - lastTurnError) / dtSec;   // degrees per second
            raw += KD * rate;
        }

        double limited = clamp(raw, MIN_TURN, MAX_TURN, turnError);
        haveLast = true;
        lastTurnError = turnError;
        lastMs = nowMs;

        return new Aim(limited, true, false, bearingDeg);
    }

    /**
     * Clamp magnitude into [MIN, MAX], keeping the sign of the request. The minimum
     * stops friction from stalling a small correction; the maximum keeps the tag in
     * frame. If the two ever cross, MAX wins -- never let the floor exceed the ceiling.
     */
    private static double clamp(double v, double min, double max, double signOf) {
        double mag = Math.abs(v);
        if (mag > max) mag = max;
        if (mag < min) mag = min;
        if (mag > max) mag = max;
        return Math.copySign(mag, signOf);
    }
}
