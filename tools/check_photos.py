"""Run the POLLEN detector over real photos, on a laptop, with no robot.

Take pictures of a yellow ball with a phone, put them in a folder, and this runs
the SAME code that runs on the Limelight over each one. You get to see what it
found, what it missed, and -- when it misses -- what colour the ball actually was
so you can fix the settings.

    ../.venv/bin/python check_photos.py ../photos --dump out

USEFUL FLAGS

    --hsv-high 42       widen the colour range (a tennis ball is greener than POLLEN)
    --ball-in 2.6       real ball diameter in inches, for the distance number
    --hfov 68           your camera's field of view in degrees -- SEE THE WARNING

WARNING ABOUT DISTANCE: the distance number is only right if --hfov matches the
camera that took the picture. The default, 82, is the Limelight's lens. A phone is
usually somewhere around 65-77. If you don't know yours, ignore the distance and
trust the "found / not found" part, which does not depend on it.
"""

import argparse
import glob
import importlib.util
import os
import sys

import cv2
import numpy as np


def load_pipeline():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "snapscript", "yellow_waffle_ball.py")
    spec = importlib.util.spec_from_file_location("yellow_waffle_ball", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def dominant_hues(image, min_sat=90, min_val=80, top=3):
    """The most common strong colours in the picture, as hue numbers.

    This is the bit that tells you WHY nothing was found. If your gate stops at 35
    and the biggest blob of colour in the photo is at 37, that is your answer.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    strong = (s >= min_sat) & (v >= min_val)
    if not np.any(strong):
        return []
    counts = np.bincount(h[strong].ravel(), minlength=180)
    # Group into 5-wide buckets so one noisy pixel doesn't win.
    buckets = [(int(counts[i:i + 5].sum()), i + 2) for i in range(0, 180, 5)]
    buckets.sort(reverse=True)
    return [(hue, n) for n, hue in buckets[:top] if n > 0]


def why_nothing(image, mod):
    """Work out WHICH setting is stopping the ball being seen.

    There are two different problems and they need opposite fixes, so it matters
    which one you have:

      wrong HUE    the ball is a different colour than the gate allows. A tennis
                   ball is greener than POLLEN. Fix with --hsv-high / --hsv-low.
      too WASHED   the hue is fine, but the ball is too pale or too dark to pass
                   the strength and brightness limits. This is what different rooms
                   and different lighting do to you. Fix with --min-sat.

    Widening the hue range to fix a lighting problem is the classic wrong move: it
    does not help, and it starts letting orange things through.
    """
    gate_lo, gate_hi = int(mod.HSV_LOW[0]), int(mod.HSV_HIGH[0])
    strict = dominant_hues(image, mod.HSV_LOW[1], mod.HSV_LOW[2])
    relaxed = dominant_hues(image, 40, 40)

    def in_gate(hs):
        return [h for h, _ in hs if gate_lo <= h <= gate_hi]

    if relaxed and in_gate(relaxed) and not in_gate(strict):
        return ("the colour is RIGHT but the ball is too pale or too dark to pass. "
                "This is a lighting problem, not a colour problem -- try "
                "--min-sat 60 before touching the hue.")
    if strict:
        top = strict[0][0]
        msg = "biggest colour blob is at hue %d" % top
        if top > gate_hi:
            return msg + ", your gate stops at %d -> try --hsv-high %d" % (gate_hi, top + 4)
        if top < gate_lo:
            return msg + ", your gate starts at %d -> try --hsv-low %d" % (gate_lo, max(0, top - 4))
        return msg + ", which is inside your gate -- so the colour is fine and " \
                     "something else rejected it. Use --dump and look at the red outlines."
    return "no strong colour anywhere in this picture. Too dark, or badly out of focus?"


def marginal_colour(image, ll, mod):
    """Warn when the detected ball's colour is close to the edge of the gate.

    Returns a message, or None if all is well. Close to the edge means you are
    probably only seeing part of the ball, and the distance will be wrong.
    """
    cx, cy, r = int(ll[4]), int(ll[5]), int(ll[6])
    if r < 1:
        return None
    mask = np.zeros(image.shape[:2], np.uint8)
    cv2.circle(mask, (cx, cy), r, 255, -1)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hues = hsv[..., 0][(mask > 0) & (hsv[..., 1] >= mod.HSV_LOW[1])]
    if hues.size == 0:
        return None
    lo, hi = int(np.percentile(hues, 5)), int(np.percentile(hues, 95))
    gate_lo, gate_hi = int(mod.HSV_LOW[0]), int(mod.HSV_HIGH[0])
    if hi >= gate_hi - 2:
        return ("ball colour reaches hue %d and your gate stops at %d -- you may be "
                "seeing only part of the ball, so the distance will read too far. "
                "Try --hsv-high %d" % (hi, gate_hi, hi + 4))
    if lo <= gate_lo + 2:
        return ("ball colour reaches hue %d and your gate starts at %d -- try "
                "--hsv-low %d" % (lo, gate_lo, max(0, lo - 4)))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="a folder of images, or one image file")
    ap.add_argument("--dump", metavar="DIR", help="write annotated copies here")
    ap.add_argument("--hsv-low", type=int, help="lowest hue to accept (default 20)")
    ap.add_argument("--hsv-high", type=int, help="highest hue to accept (default 35)")
    ap.add_argument("--min-sat", type=int, help="minimum colour strength (default 90)")
    ap.add_argument("--ball-in", type=float, help="real ball diameter in INCHES")
    ap.add_argument("--hfov", type=float, help="camera field of view, degrees (default 82)")
    args = ap.parse_args()

    mod = load_pipeline()

    if args.hsv_low is not None:
        mod.HSV_LOW[0] = args.hsv_low
    if args.hsv_high is not None:
        mod.HSV_HIGH[0] = args.hsv_high
    if args.min_sat is not None:
        mod.HSV_LOW[1] = args.min_sat
    if args.ball_in is not None:
        mod.BALL_DIAMETER_M = args.ball_in * 0.0254
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

    print("colour gate: hue %d-%d, strength >= %d, brightness >= %d"
          % (mod.HSV_LOW[0], mod.HSV_HIGH[0], mod.HSV_LOW[1], mod.HSV_LOW[2]))
    print("ball diameter: %.2f in    camera fov: %.0f deg"
          % (mod.BALL_DIAMETER_M / 0.0254, mod.HFOV_DEG))
    print()

    found = 0
    for f in files:
        img = cv2.imread(f)
        if img is None:
            print("%-30s  could not read this file" % os.path.basename(f))
            continue

        _, annotated, ll = mod.runPipeline(img.copy(), [])
        name = os.path.basename(f)

        if ll[0]:
            found += 1
            how = "" if ll[0] == 1 else "  (found the harder way, see README)"
            print("%-30s  FOUND   radius %5.1f px   %.2f m away%s"
                  % (name, ll[6], ll[3], how))
            # "Found" is not the same as "found correctly". If the ball's colour sits
            # near the edge of the gate, only part of it passes, the detector locks
            # onto that sliver, and it reports a confident distance that is far too
            # large. Measured: a ball of radius 70 px came back as 15.8 px that way.
            # So check what colour actually got through, and say when it is marginal.
            warn = marginal_colour(img, ll, mod)
            if warn:
                print("%-30s  WARNING: %s" % ("", warn))
        else:
            print("%-30s  nothing found.  %s" % (name, why_nothing(img, mod)))

        if args.dump:
            cv2.imwrite(os.path.join(args.dump, name), annotated)

    print("\nfound the ball in %d of %d picture(s)" % (found, len(files)))
    if args.dump:
        print("annotated copies written to %s" % args.dump)
        print("green circle = found.  red outline = looked at it and said no.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
