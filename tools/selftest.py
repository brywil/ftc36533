"""Run the SnapScript against synthetic frames. No camera, no Limelight.

    ../.venv/bin/python selftest.py [--dump DIR]

Checks, in order:
  1. detection across a range of apparent ball sizes (i.e. across distance)
  2. that the recovered center and radius are accurate despite the lattice
  3. that the distractors are rejected (tape, orange ball, specks, bumper slab)
  4. how the detection depends on CLOSE_K vs the lattice hole size
"""

import argparse
import importlib.util
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import synth_waffle


def load_pipeline():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "snapscript", "yellow_waffle_ball.py")
    spec = importlib.util.spec_from_file_location("yellow_waffle_ball", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def run(mod, frame):
    contour, image, llpython = mod.runPipeline(frame.copy(), [])
    return llpython, image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", metavar="DIR", help="write annotated frames here")
    args = ap.parse_args()
    if args.dump:
        os.makedirs(args.dump, exist_ok=True)

    mod = load_pipeline()
    W, H = 640, 480
    f = (W / 2.0) / np.tan(np.radians(mod.HFOV_DEG) / 2.0)
    failures = []

    # ---- 1 + 2. size sweep, center and radius accuracy -------------------
    print("=" * 74)
    print("SIZE SWEEP  (640x480, focal %.1f px, ball %.4f m)" % (f, mod.BALL_DIAMETER_M))
    print("%6s %9s %7s %9s %9s %7s %9s" %
          ("r_true", "range_m", "found", "ctr_err", "r_err_%", "circ", "dist_err"))
    print("-" * 74)
    for r in (10, 14, 20, 30, 45, 60, 90, 130):
        cx, cy = 300.0, 210.0
        frame, _ = synth_waffle.make_frame(W, H, cx, cy, r, seed=r)
        ll, ann = run(mod, frame)
        true_range = mod.BALL_DIAMETER_M * f / (2.0 * r)
        if not ll[0]:
            print("%6d %9.2f %7s %9s %9s %7s %9s" % (r, true_range, "MISS", "-", "-", "-", "-"))
            failures.append("miss at r=%d (range %.2f m)" % (r, true_range))
            continue
        ctr_err = float(np.hypot(ll[4] - cx, ll[5] - cy))
        r_err   = 100.0 * (ll[6] - r) / r
        d_err   = 100.0 * (ll[3] - true_range) / true_range
        print("%6d %9.2f %7s %9.2f %9.1f %7.2f %8.1f%%" %
              (r, true_range, "yes", ctr_err, r_err, ll[7], d_err))
        # Scale the tolerance with the target: 3 px on a 130 px ball is 2% of the
        # radius, on a 10 px ball it is 30%. An absolute threshold tests neither.
        ctr_tol = max(2.0, 0.05 * r)
        if ctr_err > ctr_tol:
            failures.append("center error %.2f px (tol %.1f) at r=%d" % (ctr_err, ctr_tol, r))
        if abs(r_err) > 12.0:
            failures.append("radius error %.1f%% at r=%d" % (r_err, r))
        if args.dump:
            cv2.imwrite(os.path.join(args.dump, "sweep_r%03d.png" % r), ann)

    # ---- 3. distractor rejection ----------------------------------------
    print("\n" + "=" * 74)
    print("DISTRACTOR REJECTION  (ball small, slab/tape larger in area)")
    print("-" * 74)
    for r in (14, 20, 30):
        cx, cy = 250.0, 260.0
        frame, _ = synth_waffle.make_frame(W, H, cx, cy, r, distractors=True, seed=99 + r)
        ll, ann = run(mod, frame)
        if not ll[0]:
            print("  r=%-4d MISS -- a distractor starved the real ball" % r)
            failures.append("distractor starvation at r=%d" % r)
        else:
            err = float(np.hypot(ll[4] - cx, ll[5] - cy))
            tag = "ok" if err < 5 else "LOCKED ON A DISTRACTOR"
            print("  r=%-4d found at (%.0f,%.0f) err %.1f px  circ %.2f  %s"
                  % (r, ll[4], ll[5], err, ll[7], tag))
            if err >= 5:
                failures.append("locked onto a distractor at r=%d" % r)
        if args.dump:
            cv2.imwrite(os.path.join(args.dump, "distract_r%03d.png" % r), ann)

    # ---- 4. CLOSE_K vs hole size ----------------------------------------
    print("\n" + "=" * 74)
    print("CLOSE_K vs LATTICE HOLE SIZE   (hole dia = %.2f x ball dia)"
          % synth_waffle.HOLE_DIAMETER_FRAC)
    print("%8s %10s %10s  %s" % ("r_true", "hole_px", "CLOSE_K", "result"))
    print("-" * 74)
    original = mod.CLOSE_K
    for r in (20, 60):
        hole_px = synth_waffle.HOLE_DIAMETER_FRAC * 2 * r
        for k in (3, 5, 7, 9, 13, 21):
            mod.CLOSE_K = k
            frame, _ = synth_waffle.make_frame(W, H, 300.0, 210.0, r, seed=7)
            ll, _ = run(mod, frame)
            if ll[0]:
                res = "found, r_err %+5.1f%%  circ %.2f" % (100.0 * (ll[6] - r) / r, ll[7])
            else:
                res = "MISS"
            print("%8d %10.1f %10d  %s" % (r, hole_px, k, res))
        print()
    mod.CLOSE_K = original

    # ---- verdict ---------------------------------------------------------
    print("=" * 74)
    if failures:
        print("FAIL (%d)" % len(failures))
        for x in failures:
            print("  - " + x)
        return 1
    print("PASS -- every check clean")
    return 0


if __name__ == "__main__":
    sys.exit(main())
