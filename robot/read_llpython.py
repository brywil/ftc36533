"""Robot-side read of the SnapScript's llpython array (robotpy / pynetworktables).

Contract, set in snapscript/yellow_waffle_ball.py:

    llpython[0]  valid flag, 1 when a ball passed every shape test
    llpython[1]  tx        degrees, + is right of crosshair
    llpython[2]  ty        degrees, + is above crosshair
    llpython[3]  distance  meters, from the apparent ball diameter
    llpython[4]  cx        px, image-space center
    llpython[5]  cy        px
    llpython[6]  radius    px, min enclosing circle of the hull
    llpython[7]  circ      0..1 shape score, useful as a confidence gate
"""

from networktables import NetworkTables

EMPTY = [0.0] * 8


class YellowBallTracker:
    def __init__(self, table="limelight", min_circ=0.75):
        self.table = NetworkTables.getTable(table)
        # Tighter than the pipeline's own MIN_CIRC: the pipeline decides what to
        # draw, the robot decides what to drive at.
        self.min_circ = min_circ

    def get(self):
        """Return (tx_deg, ty_deg, distance_m) or None when there is no good target."""
        d = self.table.getNumberArray("llpython", EMPTY)
        if len(d) < 8 or not d[0]:
            return None
        if d[7] < self.min_circ:
            return None
        return d[1], d[2], d[3]


if __name__ == "__main__":
    import time

    NetworkTables.initialize(server="10.0.0.2")   # <-- your roboRIO / limelight address
    tracker = YellowBallTracker()
    while True:
        target = tracker.get()
        print("no target" if target is None
              else "tx %+6.2f  ty %+6.2f  dist %.2f m" % target)
        time.sleep(0.1)
