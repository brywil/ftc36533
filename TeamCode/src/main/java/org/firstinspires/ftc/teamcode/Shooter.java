package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

/**
 * The flywheel shooter, driven by the distance the Limelight reports to a target's
 * AprilTag.
 *
 * Robot Configuration names expected on the Control Hub:
 *
 *     flywheel        feed            (feed is optional)
 *
 * HOW RANGE TURNS INTO POWER -- and why there is no formula here.
 *
 * Ball range depends on exit speed, and exit speed depends on wheel speed, but the
 * chain between them is not clean: the wheel slips against the ball, the ball
 * launches at some angle, drag acts on it, and the target sits some height above
 * the muzzle. A derived formula would need every one of those measured anyway, and
 * getting one wrong makes the robot confidently wrong -- which is exactly the
 * failure this team already decided to avoid (see README, "a number being written
 * down is not evidence").
 *
 * So the mapping is a TABLE of measured points, and between them we interpolate.
 * You produce the table with ShooterCalibrateOpMode: stand at a distance, find the
 * power that lands the ball where you want, record it. Four or five points from
 * your closest shot to your farthest cover a field.
 *
 * NO ENCODER. The flywheel is open-loop, so the same power gives a slightly
 * different wheel speed as the battery drains over a match. The table is a best
 * fit, not a guarantee; re-run a calibration point late in a pack to see the drift.
 * If an encoder is added later, the natural upgrade is to table wheel *velocity*
 * instead of power -- ShooterAPI is shaped so that is the only thing that changes.
 *
 * FAIL CLOSED. Until MIN_POINTS are recorded, distanceFor(power) has no answer and
 * rangeToPower() returns null; the OpModes refuse to fire rather than spray at a
 * guessed power. A missing flywheel motor is equally harmless -- every motor is
 * looked up with tryGet, and a null motor simply does nothing.
 */
public class Shooter {

    public static final String FLYWHEEL = "flywheel";
    public static final String FEED     = "feed";

    /**
     * Flip if the flywheel spins the wrong way. Which way is "right" depends on how
     * the wheel is mounted, so this is the one-line fix, not a rebuild.
     */
    private static final boolean FLYWHEEL_REVERSE = false;
    private static final boolean FEED_REVERSE      = false;

    /**
     * Measured points, nearest first. Each pair is {distance in inches, flywheel
     * power 0..1}. REPLACE THIS with what ShooterCalibrateOpMode prints -- these
     * four are placeholders that only make the interpolation runnable, and they are
     * deliberately monotonic so the shape is sane.
     */
    public static final double[][] RANGE_POWER_TABLE = {
            // { inches, power }
            { 24.0, 0.45 },
            { 48.0, 0.60 },
            { 72.0, 0.75 },
            { 96.0, 0.90 },
    };

    /** Fewer measured points than this and rangeToPower() refuses to answer. */
    public static final int MIN_POINTS = 2;

    /** Feed motor power while a ball is being pushed into the wheel. */
    public static final double FEED_POWER = 1.0;

    private final DcMotor flywheel;
    private final DcMotor feed;

    public Shooter(HardwareMap hardwareMap) {
        flywheel = hardwareMap.tryGet(DcMotor.class, FLYWHEEL);
        feed     = hardwareMap.tryGet(DcMotor.class, FEED);

        if (flywheel != null) {
            flywheel.setDirection(FLYWHEEL_REVERSE
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            // FLOAT, not BRAKE: a flywheel that brakes to a stop every time you
            // release the trigger chews the motor and the battery and cannot be
            // re-triggered quickly. Coasting down is the correct behavior here.
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            // Open-loop. RUN_WITHOUT_ENCODER is honest about there being no encoder
            // and avoids the SDK's velocity loop chasing a target it cannot measure.
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (feed != null) {
            feed.setDirection(FEED_REVERSE
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            // BRAKE: a feed motor holding back a stack of balls should not coast.
            feed.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            feed.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** True when the flywheel exists in the Robot Configuration. */
    public boolean hasFlywheel() { return flywheel != null; }

    /** True when the feed motor exists in the Robot Configuration. */
    public boolean hasFeed() { return feed != null; }

    /** Raw flywheel power, -1..1. Not what you want during a match -- use rangeToPower. */
    public void setPower(double power) {
        if (flywheel != null) flywheel.setPower(clamp(power));
    }

    public void stopFlywheel() { setPower(0); }

    /** Run the feed motor to push a ball into the spinning wheel. */
    public void setFeed(double power) {
        if (feed != null) feed.setPower(clamp(power) * FEED_POWER);
    }

    public void stopFeed() { setFeed(0); }

    /** Everything off. Call when the OpMode ends. */
    public void stop() {
        stopFlywheel();
        stopFeed();
    }

    /**
     * The flywheel power this distance needs, by linear interpolation of the
     * measured table. Distance beyond the table is clamped to the nearest endpoint
     * rather than extrapolated -- extrapolating past the farthest measured point is
     * how a shot becomes a guess.
     *
     * @return power 0..1, or null when the table is too small to trust.
     */
    public Double rangeToPower(double distanceIn) {
        if (RANGE_POWER_TABLE.length < MIN_POINTS) return null;

        // Table is assumed sorted nearest-first; clamp outside its span.
        if (distanceIn <= RANGE_POWER_TABLE[0][0]) return RANGE_POWER_TABLE[0][1];
        int last = RANGE_POWER_TABLE.length - 1;
        if (distanceIn >= RANGE_POWER_TABLE[last][0]) return RANGE_POWER_TABLE[last][1];

        for (int i = 0; i < last; i++) {
            double d0 = RANGE_POWER_TABLE[i][0];
            double d1 = RANGE_POWER_TABLE[i + 1][0];
            if (distanceIn >= d0 && distanceIn <= d1) {
                double t = (distanceIn - d0) / (d1 - d0);
                double p0 = RANGE_POWER_TABLE[i][1];
                double p1 = RANGE_POWER_TABLE[i + 1][1];
                return p0 + t * (p1 - p0);
            }
        }
        return RANGE_POWER_TABLE[last][1];
    }

    /** Human-readable copy of the table, for telemetry and for paste-back into code. */
    public static String tableAsJava() {
        StringBuilder sb = new StringBuilder();
        sb.append("public static final double[][] RANGE_POWER_TABLE = {\n");
        for (double[] row : RANGE_POWER_TABLE) {
            sb.append(String.format(Locale.US, "        { %.1f, %.2f },\n", row[0], row[1]));
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
