# Getting Started — Team 36533

This guide is for the people driving and programming the robot. It assumes you have
never used any of this before. If a word looks made-up, check **Words You'll See**
below — it probably is made-up, and we explain it there.

Read this on a laptop next to the robot, not on your phone on the bus. You'll be
pressing things as you go.

---

## What our robot does

This season's game is **BIOBUZZ**. The robot has two jobs:

1. **Drive around** — we have two robots: a practice bot with *mecanum* wheels (which
   can slide sideways) and a kitbot with *tank* drive (which cannot).
2. **Find POLLEN** — the yellow balls. A camera looks for them and tells the robot
   where they are.

Which robot you're on changes one part of the setup: the names to give the drive motors,
and which OpMode you run. Both are explained in Part 2. Everything else is the same.

The code in this folder does both. There is also some newer code for finding the
**HIVE** (the big tipping thing in the middle of the field) so the robot can work out
where it is standing. That part is not finished yet.

---

## Words you'll see

You do not need to memorize these. Come back when you hit one.

| Word | What it means |
|---|---|
| **OpMode** | One program the robot can run. You pick it from a list on the Driver Station. |
| **TeleOp** | An OpMode where *you* drive with the controller. |
| **Auto** | An OpMode where the robot drives itself for the first 30 seconds. |
| **Driver Station** | The phone or tablet the drivers hold. It shows the list of OpModes. |
| **Control Hub** | The box on the robot that runs the code. |
| **Robot Configuration** | A list on the Driver Station of what's plugged in where, and what each thing is *named*. |
| **mecanum** | Wheels with rollers at an angle. Lets the robot slide sideways without turning. |
| **tank / skid-steer** | One motor drives each side, left and right. To turn, the two sides run at different speeds. Can't slide sideways. |
| **strafe** | Sliding sideways. |
| **field-centric** | "Forward" means away from the driver, no matter which way the robot is pointing. |
| **robot-centric** | "Forward" means the direction the robot's nose is pointing. |
| **IMU** | A chip inside the Control Hub that knows which way the robot is facing. |
| **AprilTag** | A black-and-white square pattern, like a chunky QR code. The camera can work out exactly where it is by looking at one. |
| **cluster** | Four AprilTags printed together on one sticker. Looking at four at once is much more accurate than one. |
| **pipeline** | A little program that runs *inside the camera* and looks for something. |
| **HSV** | A way of describing colour as Hue (which colour), Saturation (how strong), Value (how bright). Easier to search for a colour with than plain red/green/blue. |
| **gradle** | The tool that turns our code into an app. You won't run it by hand — `./build.sh` does. |
| **APK** | The finished app file that gets sent to the robot. |

---

## What you need before you start

- A laptop (Windows, Mac, or Linux)
- The Control Hub, charged and switched on
- The Driver Station phone/tablet
- A gamepad
- **The robot up on blocks** for the first tests, so the wheels spin in the air

---

## Part 1 — Get the code onto your laptop

Open a terminal and type these, one line at a time, pressing Enter after each:

```bash
git clone git@github.com:brywil/ftc36533.git
cd ftc36533
./install.sh
```

The last one takes a while — maybe 10–20 minutes the first time. It is downloading
the tools that turn our code into a robot app. It prints what it's doing. **Leave it
alone until it says `Done`.**

If it stops with a red `error:` line, that line says what went wrong. Don't guess —
copy the whole line and ask a mentor.

Then, to build the app:

```bash
./build.sh
```

When it works, the last line says `APK:` and a long file path. That file is the app.

---

## Part 2 — Name everything correctly

**This is the step that breaks most often, so do it carefully.**

The code looks for parts *by name*. If the code asks for `front_left` and the
configuration says `frontLeft`, the robot won't start — it will show an error about a
missing device instead.

**First, which robot are you on?** We have two, and they need *different names*.

- **Practice bot** — mecanum wheels (it can slide sideways). Use the first table.
- **Kitbot** — tank drive (goBILDA StarterBot, one motor per side). Use the second table.

If you're not sure: count the drive motors. Four motors across four wheels, with the
rollers set at an angle, is mecanum. Two motors, one per side, is tank.

**Practice bot (mecanum) — name these exactly.** Capital letters and underscores matter:

| What it is | Name it exactly |
|---|---|
| Front left motor | `front_left` |
| Front right motor | `front_right` |
| Back left motor | `back_left` |
| Back right motor | `back_right` |
| Intake motor | `intake` |
| Left lift motor | `lift_left` |
| Right lift motor | `lift_right` |
| The Control Hub's built-in IMU | `imu` |
| The camera (if you have one plugged in) | `Webcam 1` |

**Kitbot (tank) — name these exactly.** Only the two drive motors differ:

| What it is | Name it exactly |
|---|---|
| Left side motor (all left wheels) | `left_drive` |
| Right side motor (all right wheels) | `right_drive` |
| Intake motor | `intake` |
| Left lift motor | `lift_left` |
| Right lift motor | `lift_right` |
| The camera (if you have one plugged in) | `Webcam 1` |

The kitbot does **not** need the `imu`. Leave it out — tank drive doesn't use it.
`left_drive` and `right_drive` are the names in goBILDA's own instructions, so if you
followed their build, they're already right.

Save the configuration and **activate** it.

---

## Part 3 — First drive, on blocks

**Wheels off the ground. Every time. No exceptions.**

A robot with a motor wired backwards will drive into a wall at full speed, and it is
never obvious which motor is wrong until you look.

Run the OpModes **in the order they're numbered**. Each one only asks you to get one
more thing right, so when something breaks you know what caused it.

### `0. Hardware Check` — does the robot agree with the code?

This one does not drive. It lists the motor names your configuration actually has,
which of them this code recognises, and whether the IMU is there. If a name is
misspelled, this is where you find out — in plain words, not a screen of red text.

Then press START and use the bumpers to pick a motor and hold **A** to spin it slowly.
Watch which wheel turns. If the wheel that turns isn't the one named on screen, two
motors are plugged into each other's ports. Fix that in the configuration, not in code.

Letting go of A stops the motor, so if something looks wrong, just take your thumb off.

### `1. Simple Drive` — make a wheel turn when a kid moves the stick

The smallest possible driving program. No IMU, no modes, no field-centric — every one
of those is another thing that can be broken on day one, and right now you only want to
prove that a joystick can turn a wheel.

It works with four motors *or* two, and it accepts either our motor names or the ones
in goBILDA's sample code. If it can't find the motors it says so and tells you to run
Hardware Check.

- Left stick — drive (and slide sideways, if you have mecanum wheels)
- Right stick left/right — turn

Top speed is capped at 50% on purpose. Fast robots break things and scare people.

**When this works, you have had a successful first session.** Everything after this is
an improvement on something that already moves.

### Then the two checks that matter

**On the practice bot (mecanum):**

1. **Push the left stick forward.** All four wheels must spin *forward*.
   - If one spins backwards, that motor is reversed. Tell a mentor which one — it's a
     one-line fix.
2. **Push the left stick right.** Looking down from above, the wheels
   should make an **X** shape.
   - If the robot tries to *spin* instead, two motors are in the wrong ports.

**On the kitbot (tank):**

1. **Push the left stick forward.** Both wheels must spin *forward*.
   - If one side spins backwards, that side's motor is reversed. Tell a mentor which
     side — it's a one-line fix.
2. **Push the left stick right.** The robot should spin **clockwise** seen from above.
   - If it turns the wrong way, that's a one-line fix too.

Only now put it on the floor.

If the practice bot drives fine forwards but crabs sideways whenever you turn, a wheel's
rollers are mounted the wrong way round. That's a **building** problem, not a code
problem — the rollers on the four wheels should form an X when you look down at the
robot.

---

## Part 4 — The real driving code

Which OpMode you run depends on which robot you're on:

- **Practice bot (mecanum):** `2. Mecanum TeleOp`. Needs all four motors **and** the IMU.
- **Kitbot (tank):** `2. Tank TeleOp (kitbot)`. Needs just the two drive motors. No IMU.

They are deliberately controlled the same way, so you can go from one robot to the other
without relearning anything:

| Control | What it does |
|---|---|
| Left stick | Drive (and slide, on the practice bot only) |
| Right stick, left/right | Turn |
| Right trigger | Slow down for lining up precisely. Squeeze harder, go slower. |
| `options` button | *(practice bot only)* Sets "forward" to whichever way the robot points now |
| `back` button | *(practice bot)* field-centric vs robot-centric; *(kitbot)* arcade vs tank steering |
| Right bumper | Run the intake. Hold it down; let go to stop. |
| Left bumper | Run the intake backwards, to clear a jam |
| D-pad up / down | Raise / lower the lift. Hold it down; let go to stop. |

The intake and lift motors are listed in `0. Hardware Check` too, so you can spin
each one on its own and see which way it turns. If the two lift motors fight each
other, tell a mentor — it's a one-line fix in `AttachmentMotors.java`. As always:
**wheels off the ground, and keep fingers clear of the lift, while testing.**

**Practice bot only — start in field-centric.** Point the robot away from you, press
`options`, then drive. Pushing the stick away from you moves the robot away from you,
even after it spins around. Most drivers find this much easier.

If the robot starts drifting the wrong way later in a match, press `options` again
while it's pointing away from you. That re-zeroes it.

The kitbot doesn't have field-centric — a tank base can only go the way it points — so
ignore the `options` button on it. Everything else is the same.

## Part 5 — The camera

Two separate things, don't mix them up:

**Finding POLLEN** runs *inside the Limelight camera*. The file is
`snapscript/ball_detector.py`. You paste it into the Limelight's web page, in the
Python tab. The robot then asks the camera "do you see a ball, and where?"

**Finding the HIVE** runs *on the Control Hub* using a regular webcam. Run
`3. HIVE Bench (AprilTag)` and point the camera at a HIVE cell's sticker.
This one is still being built and needs measuring before it's useful — a mentor should
be with you for it.

You can test the POLLEN finder on a laptop with no camera at all:

```bash
cd tools
../.venv/bin/python selftest.py
```

It makes fake pictures of a ball and checks the code finds it. It should print
`PASS`. This is handy for checking you haven't broken anything before going to the
field.

---

## Practising the camera on a tennis ball

You don't need POLLEN, a robot, or even the Limelight to learn this. You need a
yellow ball, a phone, and a laptop.

A tennis ball is a good stand-in. It is 2.6 inches across and POLLEN is 2.8, so it's
nearly the right size. It is a bit *greener* than POLLEN, which matters — see below.

**1. Take pictures.** Ten or so of the ball on the floor, from different distances and
in different light. Include some bad ones: far away, in shadow, half behind something.
The bad ones are the useful ones.

**2. Put them in a folder** on your laptop, then run:

```bash
cd tools
../.venv/bin/python check_photos.py ../photos --dump out
```

This runs the exact same code that runs inside the Limelight camera, and tells you
what it found in each picture.

**3. When it misses, it tells you which setting is wrong.** This is the important
part, because there are two completely different problems and they need opposite
fixes:

| What it says | What's wrong | What to change |
|---|---|---|
| "biggest colour blob is at hue 37, your gate stops at 35" | The ball is a **different colour** than the code allows | `--hsv-high 41` |
| "the colour is RIGHT but the ball is too pale or too dark" | A **lighting** problem, not a colour problem | `--min-sat 60` |

**Do not widen the hue range to fix a lighting problem.** It won't help, and it starts
letting orange things through — which on a real field means chasing the wrong object.

**4. Watch out for a "found" that's wrong.** If the ball's colour sits right at the
edge of the range, only part of the ball gets through, the code locks onto that sliver,
and it reports a distance that is far too big. We measured a ball that should have read
70 pixels across coming back as 15.8 — a distance **4.5 times too far**, with no error
message. The tool prints a WARNING when it spots this. Take the warning seriously: a
confidently wrong number is worse than no number.

**5. Check the distance with a tape measure.** Put the ball exactly 1 metre away and
see what the tool says. Add `--ball-in 2.6` so it knows a tennis ball is smaller than
POLLEN.

> The distance is only right if the camera's field of view is set correctly. The
> default (82 degrees) is the Limelight's lens, not a phone's. For phone pictures add
> `--hfov 68` or thereabouts. If you don't know your phone's, ignore the distance and
> trust the found/not-found part, which doesn't depend on it.

**6. Remember you'll do this again.** POLLEN is a slightly different yellow, and every
gym has different lighting, so these numbers will need redoing on the real thing. That
is normal and it is not a sign you did it wrong. **The skill you're practising is the
process, not the numbers** — and the process is exactly the same on the real ball.

Write down the numbers that worked, and where you were when they worked. "hsv-high 41,
tennis ball, cafeteria, lights on" is a useful note. "41" on its own is not.

---

## When something goes wrong

| What you see | What it usually means | What to do |
|---|---|---|
| OpMode isn't in the list | The app didn't get installed, or it didn't build | Run `./build.sh` again and read the last few lines |
| "Could not find the drive motors" | Names don't match | Run `0. Hardware Check` — it shows what names you actually have |
| "Unable to find a hardware device with name..." | A name in the configuration doesn't match the code | Check Part 2, character by character |
| Robot drives, but sideways is wrong | Two motors swapped in the configuration | Swap them in the configuration |
| One wheel spins the wrong way | That motor needs reversing in code | Ask a mentor, note *which* wheel |
| Robot crabs when turning | A mecanum wheel is built on the wrong corner | Look down at the robot: rollers should form an X |
| Tank robot won't turn, or turns the wrong way | A side's direction constant is wrong | Ask a mentor; it's in `TankDrivebase.java` |
| Camera sees nothing | Lighting changed, or the colour settings need adjusting | See "Numbers you can change" |
| Everything was fine yesterday | Someone changed something | `git status` shows what changed |

**The most useful habit:** when something breaks, write down *exactly* what you did
just before. "It stopped working" is very hard to help with. "It stopped working after
we changed CLOSE_K to 3" gets fixed in a minute.

---

## Numbers you can change (and ones to leave alone)

**Safe to experiment with**, in `snapscript/ball_detector.py`:

- `HSV_LOW` / `HSV_HIGH` — which colours count as "yellow". Widen these if the camera
  misses balls in dim light.
- `CLOSE_K` — POLLEN balls have holes in them, like a wiffle ball. This number fills
  the holes in so the camera sees one round ball instead of lots of little bits. Too
  small and it misses far-away balls; too big and two balls next to each other merge
  into one.

In `MecanumDrivebase.java`:

- `STRAFE_GAIN` — mecanum wheels slide sideways more weakly than they drive forward,
  so this gives sideways a boost. Drive a taped square on the floor; if the sideways
  legs come out short, raise it a little.

In `MecanumTeleOp.java` or `TankTeleOp.java`:

- `CREEP_SCALE` — how slow the precision trigger makes you go.

In `TankDrivebase.java` (kitbot only):

- `LEFT_REVERSE` / `RIGHT_REVERSE` — flip one if a side drives backwards when you push
  the stick forward. Test on blocks.

**Leave these alone unless a mentor is with you** — they were measured, not guessed,
and changing them makes the robot confidently wrong rather than obviously broken:

- `BALL_DIAMETER_M` — the real size of a POLLEN ball, from the rulebook. The robot
  works out distance from this. A wrong value means every distance is wrong.
- Anything in `HiveGeometry.java` marked as coming from the manual.

---

## Before you change code

1. Make sure everything works *now*, so you know your change caused any breakage.
2. Change **one thing** at a time.
3. Test it.
4. If it works, tell someone so it gets saved. If it doesn't, change it back.

Rule: **if it isn't saved to GitHub, it doesn't exist.** A laptop can die, and the
robot only runs what was built. Ask a mentor to help you commit and push.

---

## Questions worth asking a mentor

These are good questions, not silly ones:

- "Why is it field-centric and not robot-centric?"
- "How does the camera know how far away the ball is?"
- "What happens if two robots see the same ball?"
- "Why do we test on blocks first?"

If you can answer those, you understand this robot better than most people who drive
one.
