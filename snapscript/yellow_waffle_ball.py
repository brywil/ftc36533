"""Limelight 3A SnapScript: POLLEN detector for FTC BIOBUZZ (2026-27).

POLLEN is a 2.8 in. yellow Gopher ResisDent ball -- a perforated lattice sphere,
which is the entire reason this is not a plain blob detector.

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
# Measured from photographs of real POLLEN, 2026-09-15, under warm indoor light.
# The ball's own surface runs hue 18-31, saturation 128-217, value 168-247.
#
# SATURATION IS DOING THE WORK HERE, NOT HUE. In that room the cream curtain and
# the wall came out at median hue 22 -- the SAME hue as the ball -- so hue cannot
# separate ball from background at all. What separates them is that the ball is
# vividly coloured and the wall is not: raising the saturation floor from 90 to
# 110 removed two thirds of the background while keeping every ball.
#
# Hue is therefore set wide, only to survive a change of lighting, and is not
# expected to reject anything.
HSV_LOW  = np.array([15, 110,  80], dtype=np.uint8)
HSV_HIGH = np.array([38, 255, 255], dtype=np.uint8)

# CLOSE_K was 9, chosen against synthetic balls whose webbing was too narrow. Real
# POLLEN measures 0.430 in. holes on a 2.855 in. ball, which leaves much more
# plastic between the holes than modelled, so the lattice survives a gentler close.
# Measured both ways: on synthetic frames at small apparent size, K of 3-5 gives 0%
# radius error while 9 inflates it 4.7% and 21 by 13.7%; on real photographs, K of 9
# invented a spurious 8 px detection that 3-7 did not. Bigger is not safer here --
# an oversized close swells the silhouette and the radius feeds distance directly.
SPECK_K  = 3      # kills isolated yellow noise BEFORE the close can bridge it to the ball
CLOSE_K  = 5      # see the note below -- measured, not guessed
CLOSE_IT = 1      # a second iteration doubles the bridging reach; measured worse
OPEN_K   = 5      # final cleanup of anything the close welded together

MIN_AREA = 120    # px^2. A ball at 3.3 m is ~314 px^2; yellow specks measure ~20.
MIN_FILL = 0.55   # contour area / convex hull area -- lattice is porous, so this is low
MIN_CIRC = 0.65   # hull area / (pi r^2) from minEnclosingCircle

# When a big yellow blob fails the shape tests, it is usually the ball FUSED to a
# same-hue object (a bumper, a wall, another ball). A Hough pass recovers the
# circle from inside the fused blob. Bounded to that case so it does not run every
# frame: the Limelight CPU cannot afford HoughCircles at full rate.
HOUGH_FALLBACK   = True
HOUGH_MIN_BLOB   = 4000    # px^2 of rejected blob before it is worth the attempt
HOUGH_MIN_FILL   = 0.55    # of the proposed disk must actually be yellow

# BIOBUZZ POLLEN, from the Section 16 glossary: "2.8 in. (7.1 cm) Gopher
# ResisDent(TM) polyethylene balls in yellow". NECTAR is the other element --
# approximately 3.6 in. (9.1 cm), red or blue -- so the hue gate above rejects it
# without any extra work. Distance scales linearly off this number.
# 2.855 in. measured with calipers across several balls, against the 2.8 in. the
# manual quotes. Moulding tolerance is real and this feeds distance linearly.
BALL_DIAMETER_M = 0.0725
HFOV_DEG        = 82.0     # LL3A stock lens

# Focal length in px, derived from frame width. Keyed on the width so that changing
# the pipeline's capture resolution cannot leave a stale value cached.
_focal_cache = {}


def _focal(width):
    if width not in _focal_cache:
        _focal_cache[width] = (width / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
    return _focal_cache[width]


def _build_mask(image):
    hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)

    # Order matters. Specks first, or the close drags them into the ball's hull and
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


def runPipeline(image, llrobot):
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]
    h, w = image.shape[:2]
    f = _focal(w)

    mask = _build_mask(image)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Score EVERY contour over the area floor, not the top N by area. Ranking by area
    # and truncating lets a yellow bumper or a strip of tape starve the real ball --
    # measured: at 1.6 m the ball was only the third-largest contour in frame.
    best = None
    best_hull = None
    largest_rejected = 0.0
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA:
            continue

        hull      = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        if hull_area <= 0:
            continue

        (cx, cy), r = cv2.minEnclosingCircle(hull)
        if r <= 1:
            continue

        fill = area / hull_area                      # porosity of the lattice
        circ = hull_area / (math.pi * r * r)         # roundness of the silhouette
        if fill < MIN_FILL or circ < MIN_CIRC:
            largest_rejected = max(largest_rejected, hull_area)
            cv2.drawContours(image, [hull], -1, (0, 0, 255), 1)   # rejected, drawn red
            continue

        if best is None or r > best[2]:
            best = (cx, cy, r, circ)
            best_hull = c

    source = 1
    if best is None:
        if not (HOUGH_FALLBACK and largest_rejected >= HOUGH_MIN_BLOB):
            return np.array([[]]), image, llpython
        rec = _hough_recover(mask, image)
        if rec is None:
            return np.array([[]]), image, llpython
        cx, cy, r, fill = rec
        best = (cx, cy, r, fill)
        best_hull = cv2.ellipse2Poly((int(cx), int(cy)), (int(r), int(r)),
                                     0, 0, 360, 10).reshape(-1, 1, 2)
        source = 2

    cx, cy, r, circ = best

    # geometry
    tx   = math.degrees(math.atan2(cx - w / 2.0, f))
    ty   = math.degrees(math.atan2(h / 2.0 - cy, f))
    dist = (BALL_DIAMETER_M * f) / (2.0 * r)

    # overlay
    color = (0, 255, 0) if source == 1 else (0, 200, 255)
    cv2.circle(image, (int(cx), int(cy)), int(r), color, 2)
    cv2.circle(image, (int(cx), int(cy)), 3, (255, 0, 255), -1)
    cv2.putText(image, "%.2fm  tx %.1f  ty %.1f%s" % (dist, tx, ty,
                "" if source == 1 else "  [hough]"),
                (int(cx - r), int(cy - r) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    llpython = [source, tx, ty, dist, cx, cy, r, circ]
    return best_hull, image, llpython
