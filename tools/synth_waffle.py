"""Synthetic waffle-ball frames, for exercising the pipeline without a camera.

This is not a substitute for real footage -- it reproduces the ONE property that
makes a waffle ball hard (a porous lattice silhouette) and none of the properties
that make a real field hard (rolling shutter, motion blur, mixed color temperature,
the ball half-occluded behind a robot). Passing here means the geometry and the
morphology are right, not that the HSV bounds are.
"""

import cv2
import numpy as np

# Fractions of ball DIAMETER. Roughly a goBILDA-style lattice ball.
HOLE_DIAMETER_FRAC = 0.13
HOLE_SPACING_FRAC  = 0.22

BACKGROUND_BGR = (72, 68, 64)     # dark gray field tile


def _hsv_bgr(h, s, v):
    px = np.array([[[h, s, v]]], dtype=np.uint8)
    return tuple(int(c) for c in cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0])


def waffle_ball(frame, cx, cy, radius, hue=28, sat=210, val=235, rng=None):
    """Paint a yellow lattice ball into `frame`. Returns the ball's alpha mask."""
    h, w = frame.shape[:2]
    d = 2.0 * radius

    ball = np.zeros((h, w), dtype=np.uint8)
    cv2.circle(ball, (int(cx), int(cy)), int(radius), 255, -1)

    # Punch the lattice.
    holes = np.zeros((h, w), dtype=np.uint8)
    hole_r  = max(1, int(round(HOLE_DIAMETER_FRAC * d / 2.0)))
    spacing = max(2 * hole_r + 1, int(round(HOLE_SPACING_FRAC * d)))
    start = int(cx - radius), int(cy - radius)
    for gy in range(start[1], int(cy + radius) + spacing, spacing):
        for gx in range(start[0], int(cx + radius) + spacing, spacing):
            cv2.circle(holes, (gx, gy), hole_r, 255, -1)
    lattice = cv2.bitwise_and(ball, cv2.bitwise_not(holes))

    # Shade it like a sphere: bright toward the upper-left light, falling off at the
    # limb. Flat color would make the HSV gate look far better than it is.
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    r_norm = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2) / max(radius, 1)
    shade  = np.clip(1.05 - 0.45 * r_norm ** 2, 0.55, 1.0)
    lit    = np.clip(1.15 - 0.9 * np.sqrt(
        ((xx - (cx - 0.35 * radius)) ** 2 + (yy - (cy - 0.35 * radius)) ** 2)) / max(radius, 1),
        0.0, 1.0)

    base = np.zeros((h, w, 3), dtype=np.float32)
    base[:] = _hsv_bgr(hue, sat, val)
    base *= shade[..., None]
    base += 45.0 * (lit ** 3)[..., None]          # specular highlight, desaturating
    base = np.clip(base, 0, 255)

    m = lattice.astype(bool)
    frame[m] = base[m].astype(np.uint8)
    return lattice


def make_frame(width=640, height=480, cx=None, cy=None, radius=60,
               distractors=True, noise=4.0, seed=0):
    rng = np.random.default_rng(seed)
    cx = width / 2.0 if cx is None else cx
    cy = height / 2.0 if cy is None else cy

    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:] = BACKGROUND_BGR

    if distractors:
        # Yellow tape: right hue, wrong shape. Must die on circularity.
        cv2.rectangle(frame, (10, height - 60), (int(width * 0.55), height - 34),
                      _hsv_bgr(28, 200, 220), -1)
        # Orange game element: right shape, wrong hue. Must die on the HSV gate.
        cv2.circle(frame, (int(width * 0.86), int(height * 0.30)), 44,
                   _hsv_bgr(11, 230, 235), -1)
        # Yellow specks: right hue and shape, too small. Must die on MIN_AREA.
        for _ in range(12):
            p = (int(rng.integers(0, width)), int(rng.integers(0, height)))
            cv2.circle(frame, p, int(rng.integers(1, 4)), _hsv_bgr(29, 205, 225), -1)
        # A big yellow-ish bumper slab: larger in area than a distant ball, so it
        # tests whether candidate ranking by area can starve the real target.
        cv2.rectangle(frame, (int(width * 0.60), 20), (int(width * 0.99), 150),
                      _hsv_bgr(26, 190, 210), -1)

    mask = waffle_ball(frame, cx, cy, radius)

    if noise > 0:
        frame = np.clip(frame.astype(np.float32) +
                        rng.normal(0, noise, frame.shape), 0, 255).astype(np.uint8)
    return frame, mask
