"""FRC-side read of the SnapScript's llpython array (robotpy / pynetworktables).

FRC ONLY. An FTC robot cannot use this -- FTC has no NetworkTables and reads the
Limelight through the SDK instead. See ftc/LimelightBallTracker.java.

Contract, set in snapscript/yellow_waffle_ball.py:

    llpython[0]  source    0 = nothing, 1 = contour path, 2 = Hough fallback
    llpython[1]  tx        degrees, + is right of crosshair
    llpython[2]  ty        degrees, + is above crosshair
    llpython[3]  distance  meters, from the apparent ball diameter
    llpython[4]  cx        px, image-space center
    llpython[5]  cy        px
    llpython[6]  radius    px, min enclosing circle of the hull
    llpython[7]  confidence 0..1 -- circularity on the contour path, disk fill on
                 the Hough path. Different measures, so they get different gates.

Source 2 means the ball was fused into a same-hue blob (a bumper, a wall, another
ball) and was recovered by a Hough pass from inside it. That is a real detection,
but it is the degraded path: gate it looser and trust its radius less.
"""

from networktables import NetworkTables

EMPTY = [0.0] * 8

SOURCE_NONE    = 0
SOURCE_CONTOUR = 1
SOURCE_HOUGH   = 2

# Tighter than the pipeline's own MIN_CIRC of 0.65. The pipeline decides what to
# draw on the stream; the robot decides what to drive at.
MIN_CONF = {SOURCE_CONTOUR: 0.75, SOURCE_HOUGH: 0.55}


class Target:
    __slots__ = ("tx", "ty", "distance_m", "radius_px", "confidence", "source")

    def __init__(self, tx, ty, distance_m, radius_px, confidence, source):
        self.tx = tx
        self.ty = ty
        self.distance_m = distance_m
        self.radius_px = radius_px
        self.confidence = confidence
        self.source = source

    @property
    def fused(self):
        """True when this came out of a same-hue blob; range is less trustworthy."""
        return self.source == SOURCE_HOUGH


class YellowBallTracker:
    def __init__(self, table="limelight"):
        self.table = NetworkTables.getTable(table)

    def get(self):
        """Return a Target, or None when there is nothing worth driving at."""
        d = self.table.getNumberArray("llpython", EMPTY)
        if len(d) < 8:
            return None

        source = int(d[0])
        if source == SOURCE_NONE:
            return None
        if d[7] < MIN_CONF.get(source, 1.0):
            return None

        return Target(d[1], d[2], d[3], d[6], d[7], source)


if __name__ == "__main__":
    import time

    NetworkTables.initialize(server="10.0.0.2")   # <-- your roboRIO / limelight address
    tracker = YellowBallTracker()
    while True:
        t = tracker.get()
        print("no target" if t is None else
              "tx %+6.2f  ty %+6.2f  dist %.2f m%s"
              % (t.tx, t.ty, t.distance_m, "  [fused]" if t.fused else ""))
        time.sleep(0.1)
