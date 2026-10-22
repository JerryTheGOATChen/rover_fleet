"""Notice when a robot has stopped getting anywhere.

Nav2 can push a robot against the lidar safety gate for about a minute before it
aborts on its own (for example at the mouth of a slot the rover barely fits into:
Nav2's costmap says it fits, the gate sees walls 0.45 m away and blocks forward
motion). The grid simulator has no such case, so fleet_agent watches for it.
"""

import math


class ProgressWatchdog:
    """stuck() is True once the robot has moved less than min_move_m for timeout_s."""

    def __init__(self, timeout_s=15.0, min_move_m=0.3):
        self.timeout_s = timeout_s
        self.min_move_m = min_move_m
        self.anchor = None      # where the robot was when it last made progress
        self.since = None       # and when

    def reset(self, pose, now):
        self.anchor, self.since = tuple(pose), now

    def stuck(self, pose, now):
        if self.anchor is None or math.dist(pose, self.anchor) >= self.min_move_m:
            self.reset(pose, now)
            return False
        return now - self.since >= self.timeout_s
