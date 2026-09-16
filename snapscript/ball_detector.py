"""Limelight 3A SnapScript: ball detector for FTC BIOBUZZ (2026-27).

Finds the three scoring balls -- POLLEN (yellow), and NECTAR in red and blue --
and reports which one it is, where it is, and how far away.

They are perforated lattice spheres, which is the entire reason this is not a
plain blob detector.

Paste the body of this file into the Python tab of a Limelight pipeline, or
upload it directly. The camera calls runPipeline() once per frame.

The lattice is the reason this is not a plain blob detector: the ball is
not a solid color patch, so a raw HSV mask comes back as a ring of disconnected
fragments. A morphological close sized to the hole diameter welds it into one
silhouette before contour finding, and the shape tests run on the convex hull
because a lattice edge has an enormous, noisy perimeter that destroys the usual
4*pi*A/P^2 circularity metric.

Every constant below was exercised against synthetic lattice frames by
tools/selftest.py -- see that file for what those frames do and do not prove.
"""

import cv2
import numpy as np
import math

# ---------------------------------------------------------------- tuning
# ---------------------------------------------------------------- the balls
#
# BIOBUZZ has two scoring elements and NECTAR comes in two colours, so this
# detector hunts three things. Each one gets its own colour gate and its own real
# diameter, because distance is worked out from apparent size.
#
# SATURATION IS DOING THE WORK, NOT HUE. Measured off real photographs: under warm
# indoor light the cream curtain and the wall came out at median hue 22 -- the SAME
# hue as a POLLEN ball. Hue cannot separate them. What separates them is that the
# ball is vividly coloured and a wall is not. Hue ranges below are set wide enough
# to survive a change of lighting; the saturation floor is what rejects things.
#
# RED IS THE HARD ONE. A red NECTAR ball and a red team shirt are the same hue and
# the same saturation -- no colour gate can tell them apart. What tells them apart
# is SHAPE: measured on our photos the shirt is a 640x422 blob and the ball is a
# small round one, so the roundness test below is what does the rejecting. This is
# why MIN_CIRC matters more than the colour numbers for red.
#
# Each entry: (id, name, list of (low, high) HSV ranges, real diameter in metres).
# Red needs TWO ranges because red sits at both ends of the hue scale (it wraps
# around from 179 back to 0), so one range cannot contain it.

POLLEN_M = 0.0725    # 2.855 in. measured with calipers; the manual says 2.8
NECTAR_M = 0.0914    # 3.6 in. per the Section 16 glossary -- NOT yet caliper checked

CLASS_POLLEN      = 1
CLASS_NECTAR_RED  = 2
CLASS_NECTAR_BLUE = 3

BALL_CLASSES = [
    (CLASS_POLLEN, "POLLEN",
     [((15, 110, 80), (38, 255, 255))], POLLEN_M),

    (CLASS_NECTAR_RED, "NECTAR-RED",
     [((0, 110, 70), (9, 255, 255)), ((168, 110, 70), (179, 255, 255))], NECTAR_M),

    # Blue NECTAR measured at hue 115-117 and is the easiest of the three: nothing
    # else in the room is blue. Its value floor is lower than the others because the
    # blue plastic is simply darker -- measured value ran 63-147.
    (CLASS_NECTAR_BLUE, "NECTAR-BLUE",
     [((100, 100, 50), (130, 255, 255))], NECTAR_M),
]

SPECK_K  = 3      # kills isolated colour noise BEFORE the close can bridge it to a ball
CLOSE_K  = 5      # see the note below -- measured, not guessed
CLOSE_IT = 1      # a second iteration doubles the bridging reach; measured worse
OPEN_K   = 5      # final cleanup of anything the close welded together

# Smallest blob worth considering, per colour, in px^2. This floor is NOT about the
# ball -- it is about how much junk shares that ball's colour, so it differs by class.
#
# Yellow has almost nothing competing with it indoors, so POLLEN can afford a low
# floor and keep its range: 120 px^2 is a ball at about 1.3 m at 640x480.
# Red has a TEAM SHIRT competing with it. Measured on our photos, a red shirt sheds
# small round fragments that pass every shape test, and one was reported as a ball
# 1.95 m away. Red therefore pays for its noise with range.
MIN_AREA_BY_CLASS = {
    1: 120,    # POLLEN
    2: 400,    # NECTAR red  -- highest floor, most clutter
    3: 200,    # NECTAR blue -- little competition, but darker and noisier than yellow
}
MIN_AREA = 120    # fallback if a class is not listed
MIN_FILL = 0.55   # contour area / convex hull area -- lattice is porous, so this is low
# hull area / (pi r^2) from minEnclosingCircle.
MIN_CIRC = 0.65

# A ball must be BOTH solid and round. This is the test that rejects a red shirt
# while keeping a red ball, and no colour gate can do that job -- a team shirt and
# a NECTAR ball are the same hue AND the same saturation.
#
# Why a product rather than two separate limits: measured on real photos, the shirt
# scores fill 0.72 and circ 0.71 -- it squeaks past both floors individually,
# because a torso cropped by the frame is passably round and passably solid. A real
# ball is strongly one or the other: the red ball measured 0.96 x 0.79 = 0.76, and
# every genuine detection scored 0.62 or better, while the shirt makes only 0.51.
# "Good at both" is what a ball is; "mediocre at both" is what a person is.
MIN_SHAPE = 0.62

# When a big yellow blob fails the shape tests, it is usually the ball FUSED to a
# same-hue object (a bumper, a wall, another ball). A Hough pass recovers the
# circle from inside the fused blob. Bounded to that case so it does not run every
# frame: the Limelight CPU cannot afford HoughCircles at full rate.
# Which colours may use the Hough recovery, and why it is not all of them.
#
# The fallback exists for a real case: two balls touching fuse into one non-round
# blob that the shape test correctly rejects, and Hough digs a circle back out of
# it. Measured on our photos it did exactly that for two blue NECTAR balls.
#
# But on RED it fabricated. A red shirt is a huge same-coloured region, and Hough
# will always find *some* circle inside one -- it invented balls in three photos
# that contained no red ball at all, including one photo of two yellow balls. There
# is no way to tell "a ball fused to something" from "an arbitrary circle carved out
# of a big region" once you are working from the colour mask alone.
#
# So it is allowed where clutter is scarce and refused where it is not. A robot that
# misses one ball among several is fine; a robot that drives at a person is not.
HOUGH_BY_CLASS = {
    1: True,     # POLLEN      -- little else is yellow
    2: False,    # NECTAR red  -- shirts, alliance markings, anything warm
    3: True,     # NECTAR blue -- little else is blue
}
HOUGH_FALLBACK   = True    # master switch; per-class table above still applies
HOUGH_MIN_BLOB   = 4000    # px^2 of rejected blob before it is worth the attempt
HOUGH_MIN_FILL   = 0.55    # of the proposed disk must be the right colour

# ---------------------------------------------------- camera calibration
#
# These are THIS camera's real measured optics, read straight off it:
#
#     curl http://limelight.local:5807/hwreport
#
# Every Limelight is calibrated at the factory and will tell you its own numbers,
# which beats any spec sheet. If you ever swap cameras, re-run that command and
# paste the new values in -- do not assume one camera matches another.
#
# Two things here are better than the usual "work it out from the field of view":
#
#  * The focal length is MEASURED, not derived. Deriving it from the 54.5 degree
#    field of view gives 621 px where the real answer is 611 -- a 1.7% distance
#    error, free to avoid.
#  * The principal point is where the lens actually points, which is NOT the middle
#    of the picture. On this camera it sits 22.5 px above centre vertically. Assume
#    the middle and every ty reading is biased by about 1 degree, permanently and
#    invisibly.
#
# Calibrated at 1280x960; scaled below to whatever resolution the pipeline runs at.
CALIB_RES_X   = 1280.0
CALIB_FOCAL_X = 1221.445
CALIB_FOCAL_Y = 1223.398
CALIB_CX      = 637.226
CALIB_CY      = 502.549

# Set this to a number ONLY when running on some other camera -- a webcam, a phone --
# where the calibration above does not apply. Then focal length is derived from the
# field of view and the lens is assumed to point at the middle of the picture.
# check_photos.py sets it via --hfov. Leave it None on the Limelight.
HFOV_OVERRIDE_DEG = None

_intrinsics_cache = {}


def _intrinsics(width, height):
    """(focal_x, focal_y, centre_x, centre_y) in pixels, for this frame size.

    Cached on the frame size, so changing the pipeline's capture resolution cannot
    leave a stale value behind.
    """
    key = (width, height, HFOV_OVERRIDE_DEG)
    if key not in _intrinsics_cache:
        if HFOV_OVERRIDE_DEG is not None:
            f = (width / 2.0) / math.tan(math.radians(HFOV_OVERRIDE_DEG) / 2.0)
            _intrinsics_cache[key] = (f, f, width / 2.0, height / 2.0)
        else:
            # Focal length and principal point both scale with resolution, since
            # they are pixel counts describing the same physical lens.
            scale = width / CALIB_RES_X
            _intrinsics_cache[key] = (CALIB_FOCAL_X * scale, CALIB_FOCAL_Y * scale,
                                      CALIB_CX * scale, CALIB_CY * scale)
    return _intrinsics_cache[key]


def _build_mask(hsv, ranges):
    """Pixels matching any of this ball's colour ranges, cleaned up.

    `ranges` is a list because red wraps around the end of the hue scale and needs
    two pieces to describe it. Everything else needs one.
    """
    mask = None
    for lo, hi in ranges:
        part = cv2.inRange(hsv, np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8))
        mask = part if mask is None else cv2.bitwise_or(mask, part)

    # Order matters. Specks first, or the close drags them into a ball's hull and
    # inflates the radius -- measured 9.2% radius error the other way round.
    if SPECK_K > 1:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (SPECK_K, SPECK_K)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (CLOSE_K, CLOSE_K)), iterations=CLOSE_IT)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (OPEN_K, OPEN_K)))
    return mask


def _hough_recover(mask, image):
    """Find a circle inside a blob that failed the shape tests. None if unconvincing."""
    blur = cv2.GaussianBlur(mask, (9, 9), 2)
    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, dp=1.5, minDist=60,
                               param1=100, param2=40, minRadius=12, maxRadius=200)
    if circles is None:
        return None

    cx, cy, r = circles[0][0]          # strongest accumulator peak
    if r <= 1:
        return None

    # Do not trust the accumulator alone -- require the proposed disk to actually be
    # yellow. Hough will happily fit a circle to an arc of a rectangle.
    probe = np.zeros(mask.shape, dtype=np.uint8)
    cv2.circle(probe, (int(cx), int(cy)), int(r), 255, -1)
    disk = float(np.count_nonzero(probe))
    if disk <= 0:
        return None
    fill = float(np.count_nonzero(cv2.bitwise_and(probe, mask))) / disk
    if fill < HOUGH_MIN_FILL:
        return None

    return float(cx), float(cy), float(r), fill


def _best_in_mask(mask, image, min_area, allow_hough):
    """Best ball-shaped candidate in one colour's mask, or None.

    Returns (cx, cy, radius, score, contour, source) where source is 1 for the
    normal contour path and 2 for the Hough recovery.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Score EVERY contour over the area floor, not the top N by area. Ranking by
    # area and truncating lets a shirt or a bumper starve the real ball -- measured:
    # the ball was only the third-largest contour in frame.
    best = None
    largest_rejected = 0.0
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        hull = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        if hull_area <= 0:
            continue
        (cx, cy), r = cv2.minEnclosingCircle(hull)
        if r <= 1:
            continue

        fill = area / hull_area                 # porosity of the lattice
        circ = hull_area / (math.pi * r * r)     # roundness of the silhouette
        if fill < MIN_FILL or circ < MIN_CIRC or fill * circ < MIN_SHAPE:
            largest_rejected = max(largest_rejected, hull_area)
            cv2.drawContours(image, [hull], -1, (0, 0, 255), 1)   # rejected, red
            continue

        if best is None or r > best[2]:
            best = (cx, cy, r, fill * circ, c, 1)

    if best is not None:
        return best

    if not (HOUGH_FALLBACK and allow_hough and largest_rejected >= HOUGH_MIN_BLOB):
        return None
    rec = _hough_recover(mask, image)
    if rec is None:
        return None
    cx, cy, r, fillfrac = rec
    poly = cv2.ellipse2Poly((int(cx), int(cy)), (int(r), int(r)), 0, 0, 360, 10)
    return (cx, cy, r, fillfrac, poly.reshape(-1, 1, 2), 2)


def runPipeline(image, llrobot):
    """Called once per frame by the Limelight.

    llrobot[0] lets the robot say WHICH ball to hunt, which also saves the camera
    a lot of work -- searching one colour is three times cheaper than searching all
    three, and the Limelight's processor is not fast:

        0 or missing  hunt all three, report the nearest
        1             POLLEN only
        2             NECTAR red only
        3             NECTAR blue only
    """
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]
    h, w = image.shape[:2]
    focal_x, focal_y, centre_x, centre_y = _intrinsics(w, h)
    # The ball's apparent radius is measured the same way in both axes, so distance
    # uses the mean. On this lens fx and fy differ by 0.16%, well under the noise in
    # the radius itself.
    focal_mean = 0.5 * (focal_x + focal_y)

    wanted = 0
    try:
        if llrobot is not None and len(llrobot) > 0:
            wanted = int(llrobot[0])
    except (TypeError, ValueError):
        wanted = 0

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Compare candidates across colours by DISTANCE, not by radius in pixels. A
    # NECTAR ball is physically bigger, so at equal distance it covers more pixels;
    # picking the biggest blob would mean always preferring NECTAR.
    winner = None
    for class_id, class_name, ranges, diameter_m in BALL_CLASSES:
        if wanted and class_id != wanted:
            continue
        mask = _build_mask(hsv, ranges)
        cand = _best_in_mask(mask, image,
                             MIN_AREA_BY_CLASS.get(class_id, MIN_AREA),
                             HOUGH_BY_CLASS.get(class_id, False))
        if cand is None:
            continue
        cx, cy, r, score, contour, source = cand
        dist = (diameter_m * focal_mean) / (2.0 * r)
        if winner is None or dist < winner[0]:
            winner = (dist, class_id, class_name, cx, cy, r, score, contour, source)

    if winner is None:
        return np.array([[]]), image, llpython

    dist, class_id, class_name, cx, cy, r, score, contour, source = winner

    # Angles are measured from where the lens actually points (the principal
    # point), not from the middle of the picture. Those are not the same place --
    # see the calibration block at the top.
    tx = math.degrees(math.atan2(cx - centre_x, focal_x))
    ty = math.degrees(math.atan2(centre_y - cy, focal_y))

    colour = (0, 255, 0) if source == 1 else (0, 200, 255)
    cv2.circle(image, (int(cx), int(cy)), int(r), colour, 2)
    cv2.circle(image, (int(cx), int(cy)), 3, (255, 0, 255), -1)
    cv2.putText(image, "%s  %.2fm  tx %.1f%s" % (
                class_name, dist, tx, "" if source == 1 else "  [hough]"),
                (int(cx - r), int(cy - r) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1)

    # Index 0 packs BOTH what it is and how it was found: class * 10 + source.
    # 11 = POLLEN by contour, 12 = POLLEN by Hough, 21/22 = red, 31/32 = blue.
    # Any non-zero value still means "something was found", so the simple check
    # a robot usually wants is just "is index 0 not zero".
    llpython = [class_id * 10 + source, tx, ty, dist, cx, cy, r, score]
    return contour, image, llpython
