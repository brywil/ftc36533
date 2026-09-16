# Setting up the Limelight 3A

How to get our ball detector running on the camera and watch it work — **on a desk,
with no robot**. You need the Limelight, its USB-C cable, and a laptop.

---

## 1. Plug it in

The Limelight 3A is powered **over USB-C**. It is *not* powered over the Ethernet
cable, and it does not need the robot.

| | |
|---|---|
| Power | 4.1–5.75 V over USB-C, 4 W max |
| Cable | USB-C (camera) to USB-A (laptop) |
| Lens | 54.5° wide × 42° tall |
| Sensor | OV5647 colour, 640×480 at 90 fps |

Plug the USB-C end into the camera and the USB-A end into your laptop. **Wait 15–20
seconds** for the green status light. It is not ready before that — if you rush to
the next step you will think it is broken.

> A laptop USB port supplies the 4 W this needs, so no separate power brick. If the
> light never goes green, try a different port or cable — some USB-C cables are
> charge-only and carry no data.

---

## 2. Open the camera's web page

In a browser, go to:

```
http://limelight.local:5801
```

That is the camera's own control panel; it runs *on the camera*, not on your laptop.

If that address doesn't load, the other supported way in is the **Limelight Hardware
Manager** application — run it, scan for devices, and pick the Limelight. (`.local`
addresses rely on a discovery service that some school networks block.)

---

## 3. Put our detector on it

The detector is one file in this repo:

```
snapscript/ball_detector.py
```

Open it in a text editor and copy **the whole thing**.

In the web page, find the pipeline settings and switch the pipeline **type** to
**Python** (Limelight calls these "SnapScript" pipelines). That reveals a code editor.
Delete the example code that's in there and paste ours in.

> Being straight with you: the official docs don't spell out exactly where these
> controls sit, and I haven't had this camera in my hands. Look for a pipeline
> type selector and a Python tab. If the layout doesn't match, poke around — you
> can't break anything, and changing pipelines is reversible.

**Changes apply instantly** — there is no deploy button. If your Python has a mistake,
the error is printed right on the web page, which is the fastest debugging loop you
will ever get.

---

## 4. Watch it work

Hold a POLLEN ball in front of the camera and look at the video stream on the page.
You should see:

- a **green circle** around the ball
- a magenta dot at its centre
- text reading something like `POLLEN 0.84m tx 3.2`

Then try:

| Try this | What should happen |
|---|---|
| Red NECTAR | Green circle, label says `NECTAR-RED` |
| Blue NECTAR | Green circle, label says `NECTAR-BLUE` |
| A red shirt or jacket | **Nothing.** Rejected — it may get a red outline |
| Two balls at once | It picks the **nearest** one |
| Move the ball left and right | `tx` goes negative on the left, positive on the right |
| Walk the ball away | The distance number grows |

**Red outlines are not errors.** They mean the detector looked at something and
decided it wasn't a ball. That is the system working.

---

## 5. Check the distance with a tape measure

This is the one number worth proving, because everything downstream trusts it.

Put a POLLEN ball exactly **1 metre** from the lens and read the distance on screen.
It should say close to 1.00 m.

If it is off by the *same proportion* at every distance — always 20% high, say — then
a constant is wrong, not the detector. The two candidates are in `ball_detector.py`:

- `POLLEN_M = 0.0725` — the ball's real width, 2.855 in., measured with calipers
- the `CALIB_*` block — this camera's own lens measurements

> **The lens numbers were wrong until 2026-09-16.** The code assumed a 82° field of
> view. The real figure is 54.5°, and that one constant made every distance read
> **41% too close** — a ball at 1.00 m reported as 0.59 m — with every angle about
> 1.5× too big. Nothing errored; the robot would just have driven to the wrong place.
>
> It is now fixed from a better source than any spec sheet — **the camera's own
> factory calibration**, which you can read yourself:
>
> ```bash
> curl http://limelight.local:5807/hwreport
> ```
>
> That reports this camera's measured focal length (1221.4 px at 1280×960) and, more
> subtly, where the lens actually points. That is **not** the middle of the picture —
> it sits 22.5 px high on this one, which is about 1° of permanent aiming error if you
> assume otherwise. Every Limelight is calibrated individually, so if you ever swap
> cameras, re-run that command and paste in the new numbers.
>
> Still check it against a tape measure. A number being written down is not evidence.

---

## 6. What range to expect

Radius in pixels, and how much of the frame the ball covers, at each distance:

| distance | POLLEN @ 640×480 | POLLEN @ 1280×960 | NECTAR @ 640×480 |
|---|---|---|---|
| 0.5 m | r 45 px | r 90 px | r 57 px |
| 1.0 m | r 23 px | r 45 px | r 28 px |
| 2.0 m | r 11 px | r 23 px | r 14 px |
| 3.0 m | r 7.5 px | r 15 px | r 9.5 px |
| 4.0 m | **too small** | r 11 px | **too small** |

So at the camera's native 640×480 you can expect roughly **3 metres** on POLLEN.
Switching to 1280×960 roughly doubles the range because the ball covers twice as many
pixels — but it costs frame rate, so only do it if you actually need the distance.

---

## 7. When it doesn't work

| What you see | Usually means | Do this |
|---|---|---|
| No green light | Not powered | Different USB port, or a cable that carries data |
| Page won't load | `.local` discovery blocked | Use the Hardware Manager app |
| Red text on the web page | A Python error | Read it — it names the line |
| Ball never found | Colour settings vs. your lighting | See below |
| Finds the ball but distance is wrong | A constant | Section 5 |
| Finds your shirt | Shape limits too loose | Tell a mentor; it's `MIN_SHAPE` |

**If it can't find the ball,** the lighting in the room is different from the lighting
the settings were tuned in. There are two different problems and they need **opposite**
fixes:

- The ball is a **different colour** than the range allows → change the hue numbers.
- The ball is **too pale or too dark** to pass → change the saturation floor.

Widening the hue range to fix a lighting problem is the classic wrong move: it doesn't
help, and it starts letting other things through. Take photos and run
`tools/check_photos.py` on your laptop — it tells you *which* of the two you have.

---

## 8. Which numbers are safe to change

**Safe to experiment with**, in `ball_detector.py`:

- the hue and saturation numbers in `BALL_CLASSES` — the colour of each ball
- `CLOSE_K` — fills in the holes in the lattice so it reads as one round ball

**Leave these alone unless a mentor is with you.** They were measured, and changing
them makes the robot confidently wrong rather than obviously broken:

- `POLLEN_M`, `NECTAR_M` — the real ball sizes
- `HFOV_DEG` — the lens
- `MIN_SHAPE` — the test that stops it chasing people

---

## 9. Telling the robot which ball to hunt

Once there *is* a robot, it can ask for one colour instead of all three — worth doing,
since searching one is about three times cheaper and this camera's processor is not
fast:

```java
tracker.setWanted(LimelightBallTracker.Ball.POLLEN);
```

See `TeamCode/.../LimelightBallTracker.java`. None of that is needed for desk testing.
