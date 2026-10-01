# Robot wiring — goBILDA BIOBUZZ StarterBot

**Generated from the code and from goBILDA's own BIOBUZZ example, not measured on your
robot.** The names below are exactly what `TeamCode/` looks up, and they match
goBILDA's official `BioBuzzStarterBotTeleop` example, so **one Robot Configuration
drives both** their sample code and ours.

The *physical port numbers* are not in the code: the Robot Configuration binds a name
to a port, so any free port works as long as the names match. The directions IS
fixed, and are called out below.

There is **no Expansion Hub** on this robot. The 4 motors exactly fill the Control
Hub's 4 motor ports, and servos use the separate servo ports.

---

## The wiring

```
                    +------------------------------------------------+
                    |                  CONTROL HUB                    |
                    |  runs the OpModes; has the built-in IMU chip    |
                    +------------------------------------------------+
   motors               M0           M1          M2          M3
                         |            |           |           |
                   left_drive   right_drive    intake     launcher
                   (all left)   (all right)   (roller)   (flywheel,
                                                           ENCODER cable)

   servos              S0              S1                S2
                         |               |                 |
                left_intake_servo  right_intake_servo   windmill
                (corner assist)    (corner assist)      (feeds balls
                                                          into launcher)

   Limelight 3A  --USB-C-->  Control Hub USB port.
                             Configure as: Ethernet Device -> Limelight 3A,
                             named "limelight".
```

That is every device: 4 motors, 3 servos, 1 camera. If you also have a webcam for the
webcam HIVE path, it plugs into another USB port and is named `Webcam 1`.

### What each device is

| name | type | what it is |
|---|---|---|
| `left_drive` | motor | left side (all left wheels) |
| `right_drive` | motor | right side (all right wheels) |
| `intake` | motor | intake roller |
| `launcher` | **motor with encoder** | the flywheel that shoots POLLEN |
| `left_intake_servo` | CR servo | pulls POLLEN from corners |
| `right_intake_servo` | CR servo | pulls POLLEN from corners |
| `windmill` | CR servo | feeds balls into the launcher |
| `limelight` | Limelight 3A | AprilTags / the ball detector |
| `imu` | built in | only needed by the mecanum practice bot; the tank base ignores it |

Motor names are required (`hardwareMap.get`); a wrong name stops the OpMode at init.
The attachments are optional (`tryGet`), so a partially built robot still drives, and
`0. Hardware Check` names what is absent.

---

## The launcher's encoder cable is not optional

The launcher runs **closed-loop**: the code asks for a wheel *speed*
(`RUN_USING_ENCODER` + `setVelocity`), and the SDK holds it using the motor's encoder.
That is why the launcher is a `DcMotorEx` in the code and not a plain `DcMotor`.

**If the encoder cable is not plugged in**, the motor will still spin, but the code
cannot read its speed, so `getVelocity()` reads 0, the spin-up guard never says
"ready", and a ball will never be fed. The kit includes encoder cables; the launcher
needs one, plugged into the port beside the motor. This is the single most likely
thing to be wrong, and it presents as "the launcher spins but never shoots".

The other three motors do not need encoders.

---

## Reversal — the fixed part of the wiring

These are the directions the code sets, matching goBILDA's kit. If a device runs the
wrong way, the fix is the constant in the class, not re-plugging (unless a motor is in
the wrong port entirely):

| device | direction | where in code |
|---|---|---|
| `right_drive` | **reversed** | `TankDrivebase.RIGHT_REVERSE` |
| `right_intake_servo` | **reversed** | `Intake.RIGHT_SERVO_REVERSE` |
| `windmill` | **reversed** | `Shooter.WINDMILL_REVERSE` |
| `left_drive`, `intake`, `launcher`, `left_intake_servo` | forward | — |

Test on blocks first: left stick forward, **both wheels must spin forward**
(`GETTING_STARTED.md` Part 3).

---

## What this diagram does NOT tell you

- **Physical port numbers** — pick any free port; the Configuration binds the name.
- **Motor/servo power, gearing, polarity** — mechanical choices, not in the code.
- **The mecanum practice bot** — a different robot (4 drive motors + IMU, reverses the
  left side). This document is the StarterBot only.
- **That it is wired right.** A matching name does not prove the right device is on
  that port. `0. Hardware Check` spins each motor *and* each servo in turn so you can
  see which one actually moves — run it first, every time.

Source for the names and directions: goBILDA's official BIOBUZZ StarterBot example
(`3200-2627-0003_example-code`), downloaded from the
[Resource Guide](https://www.gobilda.com/ftc-starter-bot-resource-guide-2026-2027-season/).
