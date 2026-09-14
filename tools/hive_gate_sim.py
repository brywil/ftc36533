"""Sweep a HIVE through its arc and check the settled-gate thresholds.

The gate in HiveTracker.java accepts a pose fix only when BOTH the tag plane's
tilt and its height above the TILES match one of the two stable endpoints. This
script sweeps the seesaw through every angle it can physically occupy and reports
where the gate opens, so the tolerances can be chosen with a known margin instead
of guessed at.

It models geometry only -- no camera noise, no detector. What it proves is that
the two endpoints are separable with room to spare, and where the dead zone falls.
Run:  ../.venv/bin/python hive_gate_sim.py
"""

import math

# BIOBUZZ Competition Manual V1, same sources as HiveGeometry.java
TILT_DEG        = 30.0     # Figure 9-10, stable endpoints at +/- this
PIVOT_H_IN      = 43.95    # Section 9.6.1
CELL_OFFSET_IN  = 18.84    # Figure 9-9
LOW_TAG_H_IN    = 25.5     # Figure 9-10, "Bottom of HIVE above TILES"
HIGH_TAG_H_IN   = LOW_TAG_H_IN + 2 * CELL_OFFSET_IN * math.sin(math.radians(TILT_DEG))

# Distance from a CELL's centre down to its tag plane, implied by the two heights
# above closing against the pivot height. Derived, not quoted -- see the note in
# HiveGeometry.HIGH_TAG_HEIGHT_IN.
CELL_CENTRE_TO_TAG_IN = PIVOT_H_IN - CELL_OFFSET_IN * math.sin(math.radians(TILT_DEG)) - LOW_TAG_H_IN

# Gates under test (must match HiveGeometry.java)
TILT_TOL_DEG  = 12.0
HEIGHT_TOL_IN = 6.0


def tag_tilt(theta_deg):
    """Tag plane angle from horizontal for the CELL that is low at theta=+TILT."""
    return -theta_deg


def tag_height(theta_deg):
    """Tag plane height above the TILES for that same CELL."""
    return (PIVOT_H_IN
            - CELL_OFFSET_IN * math.sin(math.radians(theta_deg))
            - CELL_CENTRE_TO_TAG_IN)


def slot_by_tilt(t):
    if abs(t - (-TILT_DEG)) <= TILT_TOL_DEG: return "LOW"
    if abs(t - (+TILT_DEG)) <= TILT_TOL_DEG: return "HIGH"
    return "UNKNOWN"


def slot_by_height(h):
    if abs(h - LOW_TAG_H_IN)  <= HEIGHT_TOL_IN: return "LOW"
    if abs(h - HIGH_TAG_H_IN) <= HEIGHT_TOL_IN: return "HIGH"
    return "UNKNOWN"


def main():
    print("derived constants")
    print("  low tag plane   %.2f in  (manual)" % LOW_TAG_H_IN)
    print("  high tag plane  %.2f in  (derived)" % HIGH_TAG_H_IN)
    print("  separation      %.2f in" % (HIGH_TAG_H_IN - LOW_TAG_H_IN))
    print("  cell centre to tag plane %.2f in" % CELL_CENTRE_TO_TAG_IN)

    print("\n%8s %9s %9s %9s %9s  %s" %
          ("arm", "tilt", "height", "by tilt", "by height", "gate"))
    print("-" * 64)
    accepted, disagreed = [], []
    step = 0.5
    theta = -TILT_DEG
    while theta <= TILT_DEG + 1e-9:
        t, h = tag_tilt(theta), tag_height(theta)
        st, sh = slot_by_tilt(t), slot_by_height(h)
        ok = (st == sh) and st != "UNKNOWN"
        if ok:
            accepted.append(theta)
        if st != sh and "UNKNOWN" not in (st, sh):
            disagreed.append(theta)
        if abs(theta % 5.0) < 1e-9:
            print("%7.1f deg %+8.1f deg %8.2f\" %9s %9s  %s" %
                  (theta, t, h, st, sh, "ACCEPT" if ok else "reject"))
        theta += step

    endpoint_band = max(accepted) - min(a for a in accepted if a > 0) if accepted else 0
    lo = [a for a in accepted if a < 0]
    hi = [a for a in accepted if a > 0]
    print("\nresults")
    print("  accepted near LOW endpoint : arm %.1f deg .. %.1f deg" % (min(lo), max(lo)))
    print("  accepted near HIGH endpoint: arm %.1f deg .. %.1f deg" % (min(hi), max(hi)))
    dead = min(hi) - max(lo)
    print("  DEAD ZONE between them     : %.1f deg of arm travel rejected" % dead)
    print("  that is %.0f%% of the %.0f deg arc" % (100.0 * dead / (2 * TILT_DEG), 2 * TILT_DEG))
    print("  tilt/height disagreements  : %d  (expect 0 -- the tests are aligned,)" % len(disagreed))
    print("                                  so disagreement only ever means error)")

    ok = dead > 0 and len(disagreed) == 0
    print("\n%s" % ("PASS -- endpoints separable, dead zone non-empty" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
