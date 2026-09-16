"""Find good exposure and gain for the Limelight, in the room you are actually in.

Lighting changes everything, and the camera cannot fix it for you. This sweeps the
settings, measures a real frame at each one, and tells you which combination gives
a usable picture at a usable frame rate -- or tells you plainly that the room is
too dark, which is a real answer and not a failure.

    ../.venv/bin/python tune_camera.py                 # sweep and report
    ../.venv/bin/python tune_camera.py --apply         # sweep, then set the winner

THE TRADE-OFF, because it is not obvious:

  exposure  how long the sensor collects light. More = brighter, BUT it directly
            caps frame rate, and it smears a moving ball into a streak.
  gain      electronic amplification. More = brighter with NO frame-rate cost,
            but it amplifies noise as well as signal, and noise makes false
            colour blobs that look like balls.

So the rule is: use the LOWEST exposure that gives a bright enough picture, and
make up the rest with gain. If you run out of gain before the picture is bright
enough, the room is too dark -- turn on lights, do not keep turning up numbers.
"""

import argparse
import json
import subprocess
import sys
import time
import urllib.request

import cv2
import numpy as np

HOST = "172.29.0.1"
BASE = "http://%s:5807" % HOST
STREAM = "http://%s:5802" % HOST

# What a well-exposed picture looks like, and what the robot needs.
GOOD_MIN, GOOD_MAX = 70, 160
MIN_FPS = 20


def get_pipe():
    return json.load(urllib.request.urlopen(BASE + "/pipeline-atindex?index=0", timeout=8))


def set_pipe(d):
    req = urllib.request.Request(BASE + "/upload-pipeline?index=0",
                                 data=json.dumps(d).encode(),
                                 headers={"Content-Type": "application/json"},
                                 method="POST")
    urllib.request.urlopen(req, timeout=10).read()


def grab():
    subprocess.run(["curl", "-sS", "-m", "6", STREAM, "-o", "/tmp/_ll_frame.bin"],
                   capture_output=True)
    d = open("/tmp/_ll_frame.bin", "rb").read()
    s = d.find(b"\xff\xd8")
    e = d.find(b"\xff\xd9", s)
    if s < 0 or e < s:
        return None
    return cv2.imdecode(np.frombuffer(d[s:e + 2], np.uint8), cv2.IMREAD_COLOR)


def fps():
    return json.load(urllib.request.urlopen(BASE + "/status", timeout=6)).get("fps", 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true", help="set the best combination found")
    args = ap.parse_args()

    pipe = get_pipe()
    original = (pipe["exposure"], pipe["lcgain"])
    print("starting from exposure=%.0f gain=%.0f\n" % original)

    print("%9s %6s | %7s | %6s | %s" % ("exposure", "gain", "bright", "fps", "verdict"))
    print("-" * 56)
    winner = None
    # Low exposure first: it is the setting with the real costs.
    for exp, gain in [(100, 30), (200, 40), (300, 50), (400, 60),
                      (600, 70), (800, 80), (1200, 80)]:
        pipe["exposure"] = float(exp)
        pipe["lcgain"] = float(gain)
        set_pipe(pipe)
        time.sleep(3.5)
        img = grab()
        f = fps()
        if img is None:
            print("%9d %6d | frame grab failed" % (exp, gain))
            continue
        bright = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[..., 2].mean()

        if bright < GOOD_MIN:
            verdict = "too dark"
        elif bright > GOOD_MAX:
            verdict = "too bright"
        elif f < MIN_FPS:
            verdict = "bright, but too slow"
        else:
            verdict = "USABLE"
            if winner is None:          # first usable = lowest exposure = least blur
                winner = (exp, gain, bright, f)
        print("%9d %6d | %7.0f | %6.1f | %s" % (exp, gain, bright, f, verdict))

    print()
    if winner:
        print("BEST: exposure=%d gain=%d  (brightness %.0f, %.0f fps)" % winner)
        print("Lowest exposure that works, so the least motion blur.")
        if args.apply:
            pipe["exposure"] = float(winner[0])
            pipe["lcgain"] = float(winner[1])
            set_pipe(pipe)
            print("\napplied.")
        else:
            print("\nRe-run with --apply to set it.")
    else:
        print("NOTHING WORKED -- and that is a real answer, not a bug.")
        print("Every setting was either too dark or too slow, which means the room")
        print("does not have enough light for this camera. Turn lights on, open")
        print("blinds, or move somewhere brighter. No number will fix darkness.")
        pipe["exposure"], pipe["lcgain"] = original
        set_pipe(pipe)
        print("\nput the camera back to exposure=%.0f gain=%.0f" % original)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
