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
 * first place, so turning harder to recover can make it worse. So a lost target
 * stops the turn. Recovery is the driver's job (or a future search mode), not this
 * class guessing.
 *
 * BUT NOT INSTANTLY, because the camera BLINKS. On a frame where the detector finds
 * nothing -- motion blur, one member of the cluster dropping out, the pipeline
 * missing a deadline -- the bearing does not exist for that frame and comes back on
 * the next one. Zeroing the turn for those frames means a turn that stops and starts
 * ~20 times a second, and that is not a slow turn, it is a STUTTER. The driver
 * reported exactly that: "it twitches to move instead of a more constant movement".
 *
 * The rest of this codebase already handles a blink this way and for the same
 * reason -- ForwardOneAimAndShootAuto holds its RANGE through one so a broken frame
 * cannot drop the wheel's speed to zero. The bearing had no such hold, which is the
 * gap this closes.
 *
 * The difference from the spin-to-find failure above is that a coast CANNOT RENEW:
 * the clock is never restarted by a coasting frame, so the turn always dies within
 * COAST_MS of the last frame that actually saw a tag. A bounded coast is a blink
 * tolerance; a search is unbounded, and this is not one.
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

    /**
     * Turn power per degree of error. The main tuning knob.
     *
     * Raised from 0.020 for "make it aim a little faster". It brings the full MAX_TURN
     * onto an error of ~10 deg instead of ~17, so the approach is brisker, and it
     * roughly doubles what the robot is asked for at the edge of the deadband (0.088
     * against 0.05) -- which is the part of the range where a skid-steer is scrubbing
     * against carpet and a small command is simply not enough to break it loose.
     *
     * MAX_TURN is deliberately unchanged, so this does NOT raise the peak rotation
     * rate and so costs no extra motion blur. If the robot now overshoots the tag and
     * hunts around it, this is the number to bring back down; see the header's 1-2-3.
     */
    public static final double KP = 0.035;

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
     * Keep it SMALL: this is a floor on every correction, so too large a value makes
     * the smallest step wider than the deadband and the robot dithers around center
     * instead of stopping. Lowered from 0.06 after that dither was seen on the kitbot.
     *
     * At the current KP this floor barely binds -- it only applies within MIN_TURN/KP
     * = 0.9 deg of the goal, which is inside DEADBAND_DEG, so the deadband always gets
     * there first. What the robot is actually asked for near the goal is KP times the
     * error, so tune THAT for a weak last few degrees, not this.
     */
    public static final double MIN_TURN = 0.03;

    /**
     * How close counts as aimed, in degrees. The camera's own angle noise is a few
     * tenths of a degree, so this must be comfortably larger or the robot can never
     * be "settled". It should also be at least as wide as the smallest turn step, so
     * the robot can stop INSIDE it rather than stepping over it. Raised from 1.5.
     */
    public static final double DEADBAND_DEG = 2.5;

    /** How long the bearing must stay inside the deadband before "settled" is true. */
    public static final long SETTLE_MS = 250;

    /**
     * How long a turn survives a target that has just gone missing, in milliseconds.
     *
     * Long enough to cover a dropped frame or two at the camera's frame rate -- and to
     * match LimelightHiveTracker's own 200 ms staleness limit, so the two agree on what
     * "the camera just blinked" means -- and short enough that it stays a blink
     * tolerance rather than becoming a search. See the header.
     *
     * Set to 0 to have a missing target stop the turn immediately, which is what this
     * did before and what makes the motion stutter.
     */
    public static final long COAST_MS = 180;

    /**
     * Maximum number of turn direction changes allowed before giving up and
     * declaring "settled". Prevents dithering when MIN_TURN is too large for
     * the deadband. Set to 0 to disable.
     */
    public static final int MAX_DIRECTION_CHANGES = 6;

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
        /**
         * True when a tag with a usable bearing is in view THIS frame. Deliberately
         * false while coasting: a caller that decides to shoot, or to give up and
         * search, off this flag must not be told there is a target when there is not.
         */
        public final boolean hasTarget;
        /** True when the bearing has stayed inside the deadband for SETTLE_MS. */
        public final boolean settled;
        /**
         * True when the target just went missing and this turn is the last one being
         * carried through the gap (see COAST_MS). Never true at the same time as
         * hasTarget, and the only case where a caller sees a non-zero turn with no
         * target in view.
         */
        public final boolean coasting;
        /** The bearing used, in degrees; NaN when there was no target. */
        public final double bearingDeg;

        Aim(double turn, boolean hasTarget, boolean settled, boolean coasting,
            double bearingDeg) {
            this.turn = turn;
            this.hasTarget = hasTarget;
            this.settled = settled;
            this.coasting = coasting;
            this.bearingDeg = bearingDeg;
        }
    }

    private boolean haveLast = false;
    private double lastTurnError = 0.0;
    /** The last turn sent, kept so a blink can be coasted through instead of zeroed. */
    private double lastTurn = 0.0;
    /** The bearing that turn was built from; reported, so a coast can be seen. */
    private double lastBearingDeg = 0.0;
    private long lastMs = 0L;
    private long centeredSinceMs = -1L;
    private int directionChanges = 0;
    private double lastTurnSign = 0.0;

    /** Forget all history. Call when enabling aim, so an old error cannot bias the first turn. */
    public void reset() {
        haveLast = false;
        lastTurnError = 0.0;
        lastTurn = 0.0;
        lastBearingDeg = 0.0;
        lastMs = 0L;
        centeredSinceMs = -1L;
        directionChanges = 0;
        lastTurnSign = 0.0;
    }

    /**
     * @param bearingDeg the target's bearing, or null when no tag is in view.
     * @param nowMs      a monotonically increasing clock, e.g. SystemClock.uptimeMillis().
     */
    public Aim update(Double bearingDeg, long nowMs) {
        if (bearingDeg == null) {
            // Target gone -- but a frame or two of that is the camera blinking, not the
            // robot having lost the tag. Carry the last turn through a brief gap and
            // then stop and drop history (see the header for why this is not a search).
            //
            // lastMs is deliberately NOT advanced here. The clock runs from the last
            // frame that really saw a tag, so a coast cannot renew itself: the turn
            // dies within COAST_MS no matter how long the target stays away.
            if (COAST_MS > 0 && haveLast && (nowMs - lastMs) <= COAST_MS) {
                return new Aim(lastTurn, false, false, true, lastBearingDeg);
            }
            haveLast = false;
            centeredSinceMs = -1L;
            return new Aim(0.0, false, false, false, Double.NaN);
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
            lastTurn = 0.0;   // an aimed robot that blinks should coast to a STOP
            lastBearingDeg = bearingDeg;
            lastMs = nowMs;
            return new Aim(0.0, true, settled, false, bearingDeg);
        }
        centeredSinceMs = -1L;

        double raw = KP * turnError;

        if (KD != 0.0 && haveLast && nowMs > lastMs) {
            double dtSec = (nowMs - lastMs) / 1000.0;
            double rate = (turnError - lastTurnError) / dtSec;   // degrees per second
            raw += KD * rate;
        }

        double limited = clamp(raw, MIN_TURN, MAX_TURN, turnError);

        double currentTurnSign = Math.signum(limited);
        if (lastTurnSign != 0.0 && currentTurnSign != 0.0 && currentTurnSign != lastTurnSign) {
            directionChanges++;
        }
        lastTurnSign = currentTurnSign;

        boolean settled = false;
        if (MAX_DIRECTION_CHANGES > 0 && directionChanges >= MAX_DIRECTION_CHANGES) {
            settled = true;
        }

        haveLast = true;
        lastTurnError = turnError;
        lastTurn = limited;
        lastBearingDeg = bearingDeg;
        lastMs = nowMs;

        return new Aim(limited, true, settled, false, bearingDeg);
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
        return Math.copySign(mag, signOf);
    }
}
