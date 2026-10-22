"""Hand-written policies to compare the learned one against.

They read the same Observation the network does (no extra information), so
the comparison is fair.

greedy       the single-robot FSM's frontier rule: cost = path length - frontier
             length. Keeps its current frontier while it lasts, and skips
             frontiers a higher-priority teammate is heading for
return       greedy, but after `period` seconds out of contact with the base,
             head back until reconnected
breadcrumb   return, plus drop a relay beacon just before leaving the radio
             range of the base / existing beacons

"Perfect comms" isn't a policy: it is greedy run with comm_range = inf.
"""

import numpy as np

from .config import MAX_FRONTIERS
from .observations import (AGE_SCALE, CLAIM_SCALE, DIST_SCALE, F_CLAIMED, F_CURRENT, F_PATH,
                           F_SIZE, S_CONNECTED, S_SIGNAL, S_SINCE_BASE, SIZE_SCALE, SLOT_BASE,
                           SLOT_DROP, SLOT_STAY)


class Greedy:
    name = 'greedy'

    def __init__(self, claim_radius_m: float = 1.5):
        self.claim_radius_m = claim_radius_m

    def act(self, obs) -> int:
        valid = obs.mask[:MAX_FRONTIERS]
        if not valid.any():   # nothing left to explore: go home
            return SLOT_BASE if obs.mask[SLOT_BASE] else SLOT_STAY
        f = obs.options[:MAX_FRONTIERS]
        cost = f[:, F_PATH] * DIST_SCALE - f[:, F_SIZE] * SIZE_SCALE
        unclaimed = valid & (f[:, F_CLAIMED] * CLAIM_SCALE >= self.claim_radius_m)
        current = unclaimed & (f[:, F_CURRENT] > 0)
        if current.any():   # stick with it: switching every 3 s gets nowhere
            return int(np.flatnonzero(current)[0])
        pool = unclaimed if unclaimed.any() else valid
        return int(np.argmin(np.where(pool, cost, np.inf)))


class ReturnToBase(Greedy):
    name = 'return'

    def __init__(self, period_s: float = 60.0, **kwargs):
        super().__init__(**kwargs)
        self.period_s = period_s

    def act(self, obs) -> int:
        s = obs.self_features
        overdue = s[S_SINCE_BASE] * AGE_SCALE >= self.period_s
        if not s[S_CONNECTED] and overdue and obs.mask[SLOT_BASE]:
            return SLOT_BASE
        return super().act(obs)


class Breadcrumb(ReturnToBase):
    name = 'breadcrumb'

    def __init__(self, drop_below: float = 0.2, **kwargs):
        super().__init__(**kwargs)
        self.drop_below = drop_below

    def act(self, obs) -> int:
        s = obs.self_features
        # Still in range of the network, but about to leave it: leave a relay behind.
        if obs.mask[SLOT_DROP] and s[S_CONNECTED] and 0.0 <= s[S_SIGNAL] < self.drop_below:
            return SLOT_DROP
        return super().act(obs)
