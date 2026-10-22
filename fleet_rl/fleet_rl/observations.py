"""What one robot knows, turned into the policy's input.

The grid simulator and the ROS fleet_agent node both call build_observation,
so the policy sees exactly the same kind of input in training and in Gazebo.

A robot's choices ("options") fill fixed slots:

    0..7    frontiers, nearest first (by path length)
    8..12   go toward a teammate's last known position (stopping 1.5 m short)
    13      return to base (stopping 1 m short)
    14      stay here (e.g. to act as a relay)
    15      drop a relay beacon here (then stay)

Each option is described by OPTION_DIM numbers; slots that don't apply are
masked out. SELF_DIM more numbers describe the robot's own situation.

Teammate and base goals stop short of the spot itself: being close is enough
for the radio, and in Gazebo two robots told to drive to the same point
would collide (and spinning in place to untangle wrecks skid-steer odometry).
"""

import math
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

from .comms import signal_margin
from .config import FREE, MAX_FRONTIERS, MAX_ROBOTS, OCCUPIED, UNKNOWN, SimConfig
from .frontiers import find_frontiers
from .grid import path_to, shortest_paths, traversable

KINDS = ('frontier', 'teammate', 'base', 'stay', 'drop')
SLOT_TEAMMATE = MAX_FRONTIERS
SLOT_BASE = SLOT_TEAMMATE + MAX_ROBOTS - 1
SLOT_STAY = SLOT_BASE + 1
SLOT_DROP = SLOT_STAY + 1
MAX_OPTIONS = SLOT_DROP + 1

# Option feature columns. Columns 0-4 are a one-hot of KINDS.
F_PATH = 5         # path length from the robot / DIST_SCALE
F_DX = 6           # target x - robot x, / DIST_SCALE
F_DY = 7           # target y - robot y, / DIST_SCALE
F_SIZE = 8         # frontier length / SIZE_SCALE
F_UNKNOWN = 9      # fraction of unknown cells within INFO_RADIUS_M of the target
F_SIGNAL = 10      # radio margin at the target (see comms.signal_margin)
F_BASE_DIST = 11   # straight-line distance target -> base / DIST_SCALE
F_CLAIMED = 12     # distance to the nearest goal of a higher-priority teammate / CLAIM_SCALE (max 1)
F_AGE = 13         # teammate options: seconds since last heard / AGE_SCALE (max 1)
F_CURRENT = 14     # 1 if this is where the robot is already heading
OPTION_DIM = 15

# Self feature columns.
S_UNREPORTED = 0   # m^2 this robot knows that the base doesn't / UNREPORTED_SCALE (max 3)
S_SINCE_BASE = 1   # seconds since last connected to base / AGE_SCALE (max 3)
S_CONNECTED = 2    # 1 if connected to base right now (directly or via relays)
S_BEACONS = 3      # beacons left / 2
S_TIME_LEFT = 4    # fraction of the mission left
S_FLEET = 5        # (fleet size - 1) / (MAX_ROBOTS - 1)
S_SIGNAL = 6       # radio margin where the robot is
S_BASE_DIST = 7    # straight-line distance to base / DIST_SCALE
S_RANGE = 8        # comm range / 12 m (infinite counts as 20 m)
S_FRONTIERS = 9    # number of frontier options / MAX_FRONTIERS
SELF_DIM = 10

DIST_SCALE = 20.0
SIZE_SCALE = 5.0
CLAIM_SCALE = 10.0
AGE_SCALE = 60.0
UNREPORTED_SCALE = 50.0
INFO_RADIUS_M = 3.0
CURRENT_TOLERANCE_M = 1.0
TEAMMATE_STANDOFF_M = 1.5
BASE_STANDOFF_M = 1.0


@dataclass
class Teammate:
    id: int
    pos: tuple                 # last known (row, col)
    goal: tuple = None         # last known target (row, col), if any
    age: float = 0.0           # seconds since last heard from


@dataclass
class AgentView:
    """Everything one robot knows. Nothing here is ground truth.

    Robots have a fixed priority (lower id = higher priority): a robot steers
    clear of goals claimed by higher-priority teammates, so two robots never
    keep swapping the same frontier back and forth.
    """

    belief: np.ndarray         # (H, W) int8: UNKNOWN / FREE / OCCUPIED
    pos: tuple                 # (row, col)
    base: tuple
    my_id: int = 0
    teammates: list = field(default_factory=list)
    beacons: list = field(default_factory=list)   # known beacon cells
    beacons_left: int = 0
    unreported_cells: int = 0
    since_base_contact: float = 0.0
    connected_to_base: bool = True
    time_left: float = 1.0
    fleet_size: int = 1
    current_target: tuple = None
    blacklist: list = field(default_factory=list)  # cells to avoid (Nav2 failed there)


@dataclass
class Observation:
    options: np.ndarray        # (MAX_OPTIONS, OPTION_DIM) float32
    mask: np.ndarray           # (MAX_OPTIONS,) bool, True = valid
    self_features: np.ndarray  # (SELF_DIM,) float32
    targets: list              # per slot: (kind, (row, col) or None)
    dist: np.ndarray = None    # path lengths (cells) from the robot, for path following
    pred: np.ndarray = None    # Dijkstra predecessors, for path following


def dead_observation():
    """Input for a robot that has failed: its only option is to stay."""
    mask = np.zeros(MAX_OPTIONS, dtype=bool)
    mask[SLOT_STAY] = True
    return Observation(np.zeros((MAX_OPTIONS, OPTION_DIM), np.float32), mask,
                       np.zeros(SELF_DIM, np.float32), [('stay', None)] * MAX_OPTIONS)


def _dist_m(a, b, cell):
    return math.hypot(a[0] - b[0], a[1] - b[1]) * cell


def _stop_short(dist, pred, start, goal, standoff_cells, width):
    """The cell on the shortest path to `goal` that is `standoff_cells` before it."""
    if dist[goal] <= standoff_cells:
        return start
    stop = dist[goal] - standoff_cells
    path = path_to(pred, start, goal, width)
    best = start
    for flat in path:
        cell = divmod(flat, width)
        if dist[cell] > stop:
            break
        best = cell
    return best


def build_observation(view: AgentView, cfg: SimConfig, escape_radius_m: float = 0.0,
                      blacklist_radius_m: float = 0.75) -> Observation:
    """Turn what a robot knows into option features, a validity mask and self features.

    escape_radius_m lets a robot plan out of a spot its own map says is too
    tight (SLAM noise in Gazebo); the grid sim leaves it at 0.
    """
    belief, cell = view.belief, cfg.cell
    h, w = belief.shape
    pos = (int(view.pos[0]), int(view.pos[1]))
    free = belief == FREE
    blocked = belief == OCCUPIED

    trav = traversable(blocked, cfg.radius_cells, free)
    if escape_radius_m > 0:
        rows, cols = np.ogrid[:h, :w]
        near = (rows - pos[0]) ** 2 + (cols - pos[1]) ** 2 <= (escape_radius_m / cell) ** 2
        trav |= free & near
    trav[pos] = True
    dist, pred = shortest_paths(trav, pos)

    window = 2 * round(INFO_RADIUS_M / cell) + 1
    unknown_frac = ndimage.uniform_filter(
        (belief == UNKNOWN).astype(np.float32), size=window, mode='constant')
    window_m2 = (window * cell) ** 2

    # Frontier options: each frontier's goal is its reachable cell nearest its centroid.
    candidates = []
    for frontier in find_frontiers(belief, min_cells=round(cfg.min_frontier_m / cell)):
        reachable = frontier.cells[np.isfinite(dist.ravel()[frontier.cells])]
        if len(reachable) == 0:
            continue
        rows, cols = np.divmod(reachable, w)
        k = np.argmin((rows - frontier.centroid[0]) ** 2 + (cols - frontier.centroid[1]) ** 2)
        goal = (int(rows[k]), int(cols[k]))
        if unknown_frac[goal] * window_m2 < cfg.min_unknown_m2:
            continue   # a pocket too small to be worth the trip
        if any(_dist_m(goal, bad, cell) < blacklist_radius_m for bad in view.blacklist):
            continue
        candidates.append((float(dist[goal]), goal, frontier.size))
    candidates.sort(key=lambda c: c[0])
    candidates = candidates[:MAX_FRONTIERS]

    targets = [(KINDS[0], None)] * MAX_OPTIONS
    sizes = np.zeros(MAX_OPTIONS)
    ages = np.zeros(MAX_OPTIONS)
    mask = np.zeros(MAX_OPTIONS, dtype=bool)
    for i, (_, goal, size) in enumerate(candidates):
        targets[i], sizes[i], mask[i] = ('frontier', goal), size, True
    mates = sorted(view.teammates, key=lambda m: m.id)[:MAX_ROBOTS - 1]
    for k, mate in enumerate(mates):
        slot = SLOT_TEAMMATE + k
        mate_pos = (int(mate.pos[0]), int(mate.pos[1]))
        ages[slot] = mate.age
        mask[slot] = bool(np.isfinite(dist[mate_pos]))
        targets[slot] = ('teammate', _stop_short(dist, pred, pos, mate_pos,
                                                 TEAMMATE_STANDOFF_M / cell, w)
                         if mask[slot] else None)
    for k in range(len(mates), MAX_ROBOTS - 1):
        targets[SLOT_TEAMMATE + k] = ('teammate', None)
    base = (int(view.base[0]), int(view.base[1]))
    mask[SLOT_BASE] = bool(np.isfinite(dist[base]))
    targets[SLOT_BASE] = ('base', _stop_short(dist, pred, pos, base, BASE_STANDOFF_M / cell, w)
                          if mask[SLOT_BASE] else None)
    targets[SLOT_STAY] = ('stay', pos)
    mask[SLOT_STAY] = True
    targets[SLOT_DROP] = ('drop', pos)
    mask[SLOT_DROP] = view.beacons_left > 0

    # Radio margin at every valid target and at the robot itself.
    valid = np.flatnonzero(mask)
    points = [targets[s][1] for s in valid] + [pos]
    margins = signal_margin(blocked, points, [tuple(view.base)] + list(view.beacons),
                            cfg.comm_range, cfg.wall_penalty, cell)
    goals = [m.goal for m in mates if m.goal is not None and m.id < view.my_id]

    options = np.zeros((MAX_OPTIONS, OPTION_DIM), dtype=np.float32)
    for slot, margin in zip(valid, margins):
        kind, target = targets[slot]
        row = options[slot]
        row[KINDS.index(kind)] = 1.0
        if kind not in ('stay', 'drop'):
            row[F_PATH] = dist[target] * cell / DIST_SCALE
        row[F_DX] = (target[1] - pos[1]) * cell / DIST_SCALE
        row[F_DY] = (target[0] - pos[0]) * cell / DIST_SCALE
        row[F_SIZE] = sizes[slot] * cell / SIZE_SCALE
        row[F_UNKNOWN] = unknown_frac[target]
        row[F_SIGNAL] = margin
        row[F_BASE_DIST] = _dist_m(target, view.base, cell) / DIST_SCALE
        claimed = min((_dist_m(target, g, cell) for g in goals), default=CLAIM_SCALE)
        row[F_CLAIMED] = min(claimed, CLAIM_SCALE) / CLAIM_SCALE
        row[F_AGE] = min(ages[slot] / AGE_SCALE, 1.0)
        if kind == 'stay':
            row[F_CURRENT] = float(view.current_target is None)
        elif kind != 'drop' and view.current_target is not None:
            row[F_CURRENT] = float(
                _dist_m(target, view.current_target, cell) < CURRENT_TOLERANCE_M)

    selff = np.zeros(SELF_DIM, dtype=np.float32)
    selff[S_UNREPORTED] = min(view.unreported_cells * cell * cell / UNREPORTED_SCALE, 3.0)
    selff[S_SINCE_BASE] = min(view.since_base_contact / AGE_SCALE, 3.0)
    selff[S_CONNECTED] = float(view.connected_to_base)
    selff[S_BEACONS] = view.beacons_left / 2.0
    selff[S_TIME_LEFT] = view.time_left
    selff[S_FLEET] = (view.fleet_size - 1) / (MAX_ROBOTS - 1)
    selff[S_SIGNAL] = margins[-1]
    selff[S_BASE_DIST] = _dist_m(pos, view.base, cell) / DIST_SCALE
    selff[S_RANGE] = min(cfg.comm_range, 20.0) / 12.0
    selff[S_FRONTIERS] = len(candidates) / MAX_FRONTIERS

    return Observation(options, mask, selff, targets, dist, pred)
