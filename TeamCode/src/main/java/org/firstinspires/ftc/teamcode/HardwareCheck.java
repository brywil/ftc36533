package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.ArrayList;
import java.util.List;

/**
 * Run this FIRST, before any driving OpMode.
 *
 * It answers one question: does the robot agree with the code about what is
 * plugged in and what it is called? That one question is behind most of the time
 * a team loses on its first day.
 *
 * It will not crash if something is missing. It tells you what it found, what it
 * expected, and what the difference is. Then it lets you spin each motor on its
 * own so you can see which wheel is which.
 *
 * WHEELS OFF THE GROUND. Put the robot on blocks before you run this.
 */
@TeleOp(name = "0. Hardware Check (start here)", group = "Bringup")
public class HardwareCheck extends LinearOpMode {

    /** The names this code hopes to find, in the order the wheels sit on the robot. */
    private static final String[][] WANTED = {
            {"front left",  "front_left",  "left_front_drive"},
            {"front right", "front_right", "right_front_drive"},
            {"back left",   "back_left",   "left_back_drive"},
            {"back right",  "back_right",  "right_back_drive"},
            {"left (2wd)",  "left_drive",  "left"},
            {"right (2wd)", "right_drive", "right"},
    };

    /**
     * Attachment motors, checked and spun the same way. Listed after the wheels.
     * Names match goBILDA's BIOBUZZ StarterBot example. There is no lift on this
     * robot, so none is listed.
     */
    private static final String[][] ATTACHMENTS = {
            {"intake",   "intake"},
            {"launcher", "launcher"},
    };

    /**
     * Continuous-rotation servos, checked the same way. They are spun with the same
     * control as the motors, because a CR servo takes power just like a motor -- so
     * this OpMode can confirm which servo turns, which is otherwise hard to see.
     */
    private static final String[] ATTACHMENT_SERVOS = {
            "left_intake_servo", "right_intake_servo", "windmill",
    };

    @Override
    public void runOpMode() {
        // foundNames and foundMotors are kept PARALLEL: only devices that were actually
        // found go into either one. Missing attachments are listed in missingNames
        // instead, because a name with nothing behind it cannot be spun.
        List<String> foundNames = new ArrayList<>();
        List<DcMotor> foundMotors = new ArrayList<>();
        List<String> missingNames = new ArrayList<>();

        // Continuous-rotation servos are collected separately and spun via the same
        // index, so one device is moving at a time whether it is a motor or a servo.
        List<String> foundServoNames = new ArrayList<>();
        List<CRServo> foundServos = new ArrayList<>();

        // What does the configuration actually contain? This is the ground truth --
        // if a name is not in this list, the code can never find it, no matter how
        // the wiring looks.
        StringBuilder inConfig = new StringBuilder();
        for (String n : hardwareMap.getAllNames(DcMotor.class)) {
            inConfig.append(n).append("  ");
        }

        // Look for each wheel under any of the names we accept. tryGet returns null
        // instead of throwing, which is the whole reason this OpMode can report a
        // problem rather than dying with a wall of red text.
        for (String[] row : WANTED) {
            DcMotor found = null;
            String usedName = null;
            for (int i = 1; i < row.length; i++) {
                DcMotor m = hardwareMap.tryGet(DcMotor.class, row[i]);
                if (m != null) { found = m; usedName = row[i]; break; }
            }
            if (found != null) {
                foundNames.add(row[0] + " = \"" + usedName + "\"");
                foundMotors.add(found);
            }
        }

        // The attachment motors go into the same list, so the bumper/A spin loop
        // below works on them with no separate code path. They are simply missing
        // from the list when the configuration does not have them yet.
        for (String[] row : ATTACHMENTS) {
            DcMotor found = null;
            String usedName = null;
            for (int i = 1; i < row.length; i++) {
                DcMotor m = hardwareMap.tryGet(DcMotor.class, row[i]);
                if (m != null) { found = m; usedName = row[i]; break; }
            }
            if (found != null) {
                foundNames.add(row[0] + " = \"" + usedName + "\"");
                foundMotors.add(found);
            } else {
                missingNames.add(row[0] + " -- name it \"" + row[1] + "\"");
            }
        }

        // Continuous-rotation servos. Spun with the same control as motors (a CR
        // servo takes power the same way), which is the only easy way to see which
        // one turns.
        for (String name : ATTACHMENT_SERVOS) {
            CRServo s = hardwareMap.tryGet(CRServo.class, name);
            if (s != null) {
                foundServoNames.add(name + " (CR servo)");
                foundServos.add(s);
            } else {
                missingNames.add(name + " -- name it \"" + name + "\"");
            }
        }

        IMU imu = hardwareMap.tryGet(IMU.class, "imu");

        telemetry.addLine("=== WHAT THE CONFIGURATION CONTAINS ===");
        telemetry.addLine(inConfig.length() == 0 ? "  (no motors at all!)" : "  " + inConfig);
        telemetry.addLine();
        telemetry.addLine("=== WHAT THIS CODE FOUND ===");
        if (foundNames.isEmpty() && foundServoNames.isEmpty()) {
            telemetry.addLine("  NOTHING.");
            telemetry.addLine("  The names in your configuration do not match any name");
            telemetry.addLine("  this code looks for. Check spelling and underscores.");
        } else {
            for (String n : foundNames) telemetry.addLine("  OK  " + n);
            for (String n : foundServoNames) telemetry.addLine("  OK  " + n);
        }
        if (!missingNames.isEmpty()) {
            telemetry.addLine();
            telemetry.addLine("=== NOT IN THE CONFIGURATION ===");
            for (String n : missingNames) telemetry.addLine("  MISSING  " + n);
            telemetry.addLine("  (drivebase still works; these are simply skipped)");
        }
        telemetry.addLine();
        telemetry.addData("imu", imu == null ? "MISSING (field-centric will not work)" : "OK");
        telemetry.addLine();
        telemetry.addLine("Wheels off the ground, then press START.");
        telemetry.update();

        waitForStart();
        int count = foundMotors.size() + foundServos.size();
        if (isStopRequested() || count == 0) return;

        int selected = 0;
        boolean leftWasPressed = false, rightWasPressed = false;

        while (opModeIsActive()) {
            // Bumpers step through the motors then the servos, one at a time.
            if (gamepad1.left_bumper && !leftWasPressed) {
                selected = (selected - 1 + count) % count;
            }
            if (gamepad1.right_bumper && !rightWasPressed) {
                selected = (selected + 1) % count;
            }
            leftWasPressed = gamepad1.left_bumper;
            rightWasPressed = gamepad1.right_bumper;

            // Hold A to spin. Letting go stops it -- so if something is wrong, the
            // fix is to take your thumb off, not to find the stop button. Motors and
            // servos share the index; only the selected one moves.
            boolean spin = gamepad1.a;
            for (int i = 0; i < foundMotors.size(); i++) {
                foundMotors.get(i).setPower(i == selected && spin ? 0.3 : 0.0);
            }
            for (int i = 0; i < foundServos.size(); i++) {
                int globalIndex = foundMotors.size() + i;
                foundServos.get(i).setPower(globalIndex == selected && spin ? 0.3 : 0.0);
            }

            telemetry.addLine("Bumpers pick a device.  Hold A to spin it slowly.");
            telemetry.addLine();
            int idx = 0;
            for (String n : foundNames) telemetry.addLine((idx++ == selected ? "  > " : "    ") + n);
            for (String n : foundServoNames) telemetry.addLine((idx++ == selected ? "  > " : "    ") + n);
            telemetry.addLine();
            telemetry.addLine(spin ? "SPINNING" : "stopped");
            telemetry.addLine();
            telemetry.addLine("Watch which wheel turns. Write it down. If the wheel that");
            telemetry.addLine("turns is not the one named above, two motors are plugged");
            telemetry.addLine("into each other's ports -- swap them in the configuration.");
            telemetry.update();
        }

        for (DcMotor m : foundMotors) m.setPower(0);
        for (CRServo s : foundServos) s.setPower(0);
    }
}
