package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Locale;

/**
 * The launcher (flywheel) that shoots POLLEN at the HIVE, plus the windmill that feeds
 * balls into it. Both are driven by the distance the Limelight reports to a target's
 * AprilTag.
 *
 * Robot Configuration names expected on the Control Hub:
 *
 *     launcher        windmill
 *
 * These match goBILDA's own BIOBUZZ StarterBot example code, so one Robot Configuration
 * drives both their sample and this code. The kit's launcher is a 5203 Yellow Jacket
 * with an encoder, so -- unlike the rest of the robot -- this mechanism is CLOSED LOOP.
 * The windmill is a continuous-rotation servo.
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
 * OpModes refuse to fire. A missing launcher or windmill is harmless -- tryGet returns
 * null and the relevant methods no-op.
 */
public class Shooter {

    public static final String LAUNCHER = "launcher";
    public static final String WINDMILL = "windmill";

    /** Flip if the wheel spins the wrong way for the ball to leave forward. */
    private static final boolean LAUNCHER_REVERSE = false;
    /** Flip if the windmill pushes balls away from the wheel instead of into it. */
    private static final boolean WINDMILL_REVERSE = true;   // goBILDA's kit reverses it

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
     * Measured points, nearest first. Each pair is {distance in inches, launcher
     * velocity in ticks per second}. REPLACE with what ShooterCalibrateOpMode prints --
     * these are placeholders that only make the interpolation runnable. goBILDA's own
     * example uses 1250 ticks/s as a full launch; the spread here is plausible, not
     * measured.
     */
    public static final double[][] RANGE_VELOCITY_TABLE = {
            // { inches, ticks per second }
            { 24.0, 1000.0 },
            { 48.0, 1150.0 },
            { 72.0, 1300.0 },
            { 96.0, 1450.0 },
    };

    /** Fewer measured points than this and rangeToVelocity() refuses to answer. */
    public static final int MIN_POINTS = 2;

    /** Ticks per second the launcher must exceed for a ball to leave cleanly. */
    public static final double MIN_LAUNCH_VELOCITY = 800.0;

    private final DcMotorEx launcher;
    private final CRServo windmill;

    // The velocity we last asked for, so isSpunUp() can be a real measurement rather
    // than a timer: has the wheel actually reached (most of) the target?
    private double commandedVelocity = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        // The launcher is a DcMotorEx because we need setVelocity/getVelocity, which
        // plain DcMotor does not have. tryGet returns null if it is missing or is
        // configured as a plain motor.
        launcher = hardwareMap.tryGet(DcMotorEx.class, LAUNCHER);
        windmill = hardwareMap.tryGet(CRServo.class, WINDMILL);

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
    }

    public boolean hasLauncher() { return launcher != null; }
    public boolean hasWindmill() { return windmill != null; }

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

    /** Run the windmill to push a ball into the wheel. +1 feed, -1 reverse, 0 stop. */
    public void setFeed(double power) {
        if (windmill != null) windmill.setPower(clamp(power));
    }

    /** Everything off. Call when the OpMode ends. */
    public void stop() {
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
