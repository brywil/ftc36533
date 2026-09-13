"""Limelight 3A SnapScript: yellow waffle-ball detector.

Paste the body of this file into the Python tab of a Limelight pipeline, or
upload it directly. The camera calls runPipeline() once per frame.

The waffle lattice is the reason this is not a plain blob detector: the ball is
not a solid color patch, so a raw HSV mask comes back as a ring of disconnected
fragments. A morphological close sized to the hole diameter welds it into one
silhouette before contour finding, and the shape tests run on the convex hull
because a lattice edge has an enormous, noisy perimeter that destroys the usual
4*pi*A/P^2 circularity metric.
"""

import cv2
import numpy as np
import math

# ---------------------------------------------------------------- tuning
HSV_LOW  = np.array([20,  90,  80], dtype=np.uint8)   # yellow: H 20-35 in OpenCV's 0-179 scale
HSV_HIGH = np.array([35, 255, 255], dtype=np.uint8)

CLOSE_K       = 9      # >= waffle hole width in px at your farthest working range
MIN_AREA      = 300    # px^2, rejects field-light glints and yellow tape specks
MIN_FILL      = 0.55   # contour area / convex hull area -- lattice is porous, so this is low
MIN_CIRC      = 0.65   # hull area / (pi r^2) from minEnclosingCircle
MAX_CANDS     = 3

BALL_DIAMETER_M = 0.1778   # <-- MEASURE YOUR BALL. 7 in placeholder; distance scales linearly off this.
HFOV_DEG        = 82.0     # LL3A stock lens

_focal_px = None           # derived from frame width on the first frame


def _focal(width):
    global _focal_px
    if _focal_px is None:
        _focal_px = (width / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
    return _focal_px


def runPipeline(image, llrobot):
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]
    h, w = image.shape[:2]
    f = _focal(w)

    # 1. color gate in HSV
    hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)

    # 2. weld the waffle lattice into one silhouette, then drop speckle
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CLOSE_K, CLOSE_K))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return np.array([[]]), image, llpython

    # 3. score candidates on shape, not just size
    best = None
    for c in sorted(contours, key=cv2.contourArea, reverse=True)[:MAX_CANDS]:
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
            cv2.drawContours(image, [hull], -1, (0, 0, 255), 1)   # rejected, drawn red
            continue

        if best is None or r > best[2]:
            best = (cx, cy, r, c, fill, circ)

    if best is None:
        return np.array([[]]), image, llpython

    cx, cy, r, contour, fill, circ = best

    # 4. geometry
    tx   = math.degrees(math.atan2(cx - w / 2.0, f))
    ty   = math.degrees(math.atan2(h / 2.0 - cy, f))
    dist = (BALL_DIAMETER_M * f) / (2.0 * r)

    # 5. overlay
    cv2.circle(image, (int(cx), int(cy)), int(r), (0, 255, 0), 2)
    cv2.circle(image, (int(cx), int(cy)), 3, (255, 0, 255), -1)
    cv2.putText(image, "%.2fm  tx %.1f  ty %.1f" % (dist, tx, ty),
                (int(cx - r), int(cy - r) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    llpython = [1, tx, ty, dist, cx, cy, r, circ]
    return contour, image, llpython
