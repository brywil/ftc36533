"""Run the ball detector over real photos, on a laptop, with no robot.

Take pictures of POLLEN or NECTAR with any camera, put them in a folder, and this
runs the SAME code that runs inside the Limelight over each one. It tells you what
it found, what it missed, and -- when it misses -- which setting is to blame.

    ../.venv/bin/python check_photos.py ../photos --dump out

USEFUL FLAGS

    --only pollen       hunt just one kind of ball (pollen, red, blue)
    --min-sat 120       raise the colour-strength floor (rejects walls and carpet)
    --hsv-low / --hsv-high    shift the hue range (see WHICH KNOB below)
    --ball-in 2.855     real ball diameter in inches, for the distance number
    --hfov 68           your camera's field of view -- SEE THE WARNING

WHICH KNOB: there are two different failures and they need opposite fixes.

    wrong HUE     the ball is a different colour than the gate allows.
    too WASHED    the hue is fine but the ball is too pale or dark to pass. This is
                  what different rooms and different lighting do to you.

Widening the hue range to fix a lighting problem is the classic wrong move: it does
not help, and it starts letting other things through. This tool tells you which one
you have, so you do not have to guess.

WARNING ABOUT DISTANCE: it is only right if --hfov matches the camera that took the
picture. The default, 82, is the Limelight's lens; a phone or webcam is usually
65-77. If you do not know yours, ignore the distance and trust found/not-found,
which does not depend on it.
"""

import argparse
import glob
import importlib.util
import os
import sys

import cv2
import numpy as np

NAMES = {"pollen": 1, "red": 2, "blue": 3}


def load_pipeline():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "snapscript", "ball_detector.py")
    spec = importlib.util.spec_from_file_location("ball_detector", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def class_entry(mod, class_id):
    for entry in mod.BALL_CLASSES:
        if entry[0] == class_id:
            return entry
    return None


def mask_for(mod, hsv, ranges):
    m = None
    for lo, hi in ranges:
        part = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        m = part if m is None else cv2.bitwise_or(m, part)
    return m


def why_nothing(image, mod, ranges):
    """Say WHICH setting is stopping the ball being seen."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    strict = mask_for(mod, hsv, ranges)
    # Same hue windows, but far more forgiving about strength and brightness.
    loose_ranges = [((lo[0], 40, 40), hi) for lo, hi in ranges]
    loose = mask_for(mod, hsv, loose_ranges)

    if cv2.countNonZero(loose) > 4 * max(cv2.countNonZero(strict), 1) and \
       cv2.countNonZero(loose) > 500:
        return ("the colour is RIGHT but the ball is too pale or too dark to pass. "
                "That is a LIGHTING problem, not a colour problem -- try a lower "
                "--min-sat before touching the hue.")

    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    strong = (s >= 90) & (v >= 60)
    if not np.any(strong):
        return "no strong colour anywhere in this picture. Too dark, or out of focus?"
    counts = np.bincount(h[strong].ravel(), minlength=180)
    buckets = sorted(((int(counts[i:i + 5].sum()), i + 2) for i in range(0, 180, 5)),
                     reverse=True)
    top = buckets[0][1]
    inside = any(lo[0] <= top <= hi[0] for lo, hi in ranges)
    if inside:
        return ("hue %d is inside the range, so the colour is fine and something "
                "else rejected it. Use --dump and look at the red outlines." % top)
    return ("the strongest colour here is hue %d, which is outside this ball's "
            "range %s -- wrong kind of ball, or the hue range needs moving."
            % (top, [(lo[0], hi[0]) for lo, hi in ranges]))


def clipping_warning(image, mod, ll, ranges):
    """Warn when the gate is CLIPPING the ball rather than containing it."""
    cx, cy, r = int(ll[4]), int(ll[5]), int(ll[6])
    if r < 3:
        return None
    circle = np.zeros(image.shape[:2], np.uint8)
    cv2.circle(circle, (cx, cy), r, 255, -1)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    passed = mask_for(mod, hsv, ranges)
    hues = hsv[..., 0][(circle > 0) & (passed > 0)]
    if hues.size < 50:
        return None
    # Only meaningful for a single non-wrapping range; red spans both ends of the
    # scale, so "the edge" is not a thing there.
    if len(ranges) != 1:
        return None
    lo, hi = int(ranges[0][0][0]), int(ranges[0][1][0])
    at_top = float((hues >= hi - 1).mean())
    at_bot = float((hues <= lo + 1).mean())
    if at_top > 0.15:
        return ("%.0f%% of the ball's colour is jammed against the TOP of the hue "
                "range (%d) -- the gate is probably cutting the ball, so the "
                "distance will read too far. Try --hsv-high %d"
                % (100 * at_top, hi, hi + 5))
    if at_bot > 0.15:
        return ("%.0f%% of the ball's colour is jammed against the BOTTOM of the hue "
                "range (%d). Try --hsv-low %d" % (100 * at_bot, lo, max(0, lo - 5)))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="a folder of images, or one image file")
    ap.add_argument("--dump", metavar="DIR", help="write annotated copies here")
    ap.add_argument("--only", choices=sorted(NAMES), help="hunt only this kind of ball")
    ap.add_argument("--hsv-low", type=int, help="lowest hue to accept")
    ap.add_argument("--hsv-high", type=int, help="highest hue to accept")
    ap.add_argument("--min-sat", type=int, help="minimum colour strength")
    ap.add_argument("--ball-in", type=float, help="real ball diameter in INCHES")
    ap.add_argument("--hfov", type=float, help="camera field of view, degrees")
    args = ap.parse_args()

    mod = load_pipeline()
    target = NAMES.get(args.only, 0)

    # Tuning flags apply to the class being hunted. With no --only they apply to
    # POLLEN, which is the one people tune most.
    tune_id = target or mod.CLASS_POLLEN
    rebuilt = []
    for cid, cname, ranges, diam in mod.BALL_CLASSES:
        if cid == tune_id:
            new = []
            for lo, hi in ranges:
                lo = list(lo); hi = list(hi)
                if args.hsv_low is not None:
                    lo[0] = args.hsv_low
                if args.hsv_high is not None:
                    hi[0] = args.hsv_high
                if args.min_sat is not None:
                    lo[1] = args.min_sat
                new.append((tuple(lo), tuple(hi)))
            ranges = new
            if args.ball_in is not None:
                diam = args.ball_in * 0.0254
        rebuilt.append((cid, cname, ranges, diam))
    mod.BALL_CLASSES = rebuilt

    if args.hfov is not None:
        mod.HFOV_DEG = args.hfov
        mod._focal_cache.clear()

    if os.path.isdir(args.path):
        files = []
        for ext in ("jpg", "jpeg", "png", "JPG", "JPEG", "PNG"):
            files += glob.glob(os.path.join(args.path, "*." + ext))
        files.sort()
    else:
        files = [args.path]
    if not files:
        print("No images found in %s" % args.path)
        return 1
    if args.dump:
        os.makedirs(args.dump, exist_ok=True)

    print("hunting: %s" % (args.only if args.only else "all three"))
    for cid, cname, ranges, diam in mod.BALL_CLASSES:
        if target and cid != target:
            continue
        print("  %-12s hue %s  sat>=%d  diameter %.2f in"
              % (cname, " and ".join("%d-%d" % (lo[0], hi[0]) for lo, hi in ranges),
                 ranges[0][0][1], diam / 0.0254))
    print()

    found = 0
    for f in files:
        img = cv2.imread(f)
        name = os.path.basename(f)
        if img is None:
            print("%-30s  could not read this file" % name)
            continue

        _, annotated, ll = mod.runPipeline(img.copy(), [target])
        if ll[0]:
            found += 1
            cid, source = int(ll[0]) // 10, int(ll[0]) % 10
            entry = class_entry(mod, cid)
            how = "" if source == 1 else "  [hough]"
            print("%-30s  %-12s radius %5.1f px   %.2f m away%s"
                  % (name, entry[1], ll[6], ll[3], how))
            warn = clipping_warning(img, mod, ll, entry[2])
            if warn:
                print("%-30s  WARNING: %s" % ("", warn))
        else:
            ranges = class_entry(mod, tune_id)[2]
            print("%-30s  nothing found.  %s" % (name, why_nothing(img, mod, ranges)))

        if args.dump:
            cv2.imwrite(os.path.join(args.dump, name), annotated)

    print("\nfound a ball in %d of %d picture(s)" % (found, len(files)))
    if args.dump:
        print("annotated copies in %s -- green circle = found, red outline = rejected"
              % args.dump)
    return 0


if __name__ == "__main__":
    sys.exit(main())
