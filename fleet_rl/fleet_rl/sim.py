"""The grid simulator: robots, lidar, radio and the base station, in 0.5 s ticks.

    sim = FleetSim(house, SimConfig(n_robots=3), seed=0)
    while not sim.done:
        observations = sim.observe()                  # one per robot
        actions = [policy.act(o) for o in observations]
        reward = sim.step(actions)                    # runs one decision period (3 s)

Each tick: robots move along their paths, their lidars update their own
maps, then robots that can reach each other over the radio (possibly through
relays) merge maps. The base station is a radio node too; what it knows is
the score.
"""

import math
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

from .comms import connected_groups, link_matrix, merge_maps
from .config import FREE, MAX_ROBOTS, OCCUPIED, UNKNOWN, SimConfig
from .grid import path_to, raycast, traversable
from .observations import (DIST_SCALE, SLOT_STAY, AgentView, Observation, Teammate,
                           build_observation, dead_observation)

ROBOT_STATE_DIM = 8
GLOBAL_DIM = 7


@dataclass
class Robot:
    id: int
    pos: tuple                          # (row, col)
    speed: float                        # m/s
    belief: np.ndarray                  # its own map (int8)
    reported: np.ndarray                # cells it knows the base has (bool)
    beacons_left: int = 0
    alive: bool = True
    fail_time: float = math.inf
    target: tuple = None
    path: list = field(default_factory=list)    # flat cell indices, next first
    progress: float = 0.0               # m travelled toward path[0]
    connected_to_base: bool = True
    last_base_contact: float = 0.0
    teammates: dict = field(default_factory=dict)   # id -> (pos, goal, time heard)
    known_beacons: set = field(default_factory=set)


class FleetSim:
    def __init__(self, house, cfg: SimConfig, seed: int = 0):
        self.house, self.cfg = house, cfg
        self.rng = np.random.default_rng(seed)
        occ = house.occupied
        self.h, self.w = occ.shape
        self.truth_trav = traversable(occ, cfg.radius_cells)
        labels, _ = ndimage.label(~occ)
        self.coverable = labels == labels[house.base]   # free space connected to the base
        self.n_coverable = int(self.coverable.sum())

        self.t = 0.0
        self.base_belief = np.full(occ.shape, UNKNOWN, dtype=np.int8)
        self.base_beacons = set()
        self.beacons = []                                # placed beacon cells
        self.discovered = np.zeros(occ.shape, dtype=bool)   # ever seen free by any robot
        self.robots = []
        for i in range(cfg.n_robots):
            self.robots.append(Robot(
                id=i, pos=tuple(house.spawns[i % len(house.spawns)]),
                speed=cfg.speed * self.rng.uniform(1.0 - cfg.speed_jitter, 1.0),
                belief=np.full(occ.shape, UNKNOWN, dtype=np.int8),
                reported=np.zeros(occ.shape, dtype=bool),
                beacons_left=cfg.beacons_per_robot))
        if cfg.n_robots > 1 and self.rng.random() < cfg.dropout_prob:
            victim = self.robots[self.rng.integers(cfg.n_robots)]
            victim.fail_time = self.rng.uniform(0.2, 0.8) * cfg.episode_seconds

        self.history = {'t': [], 'base': [], 'team': []}
        self.links = np.zeros((0, 0), dtype=bool)        # last tick's radio links (for drawing)
        self.on_tick = None                              # optional callback(sim), e.g. a recorder
        self._observations = None
        self._sense()
        self._communicate()
        self._record()

    # ------------------------------------------------------------ public

    @property
    def done(self) -> bool:
        return (self.t >= self.cfg.episode_seconds - 1e-9
                or not any(r.alive for r in self.robots)
                or self.history['base'][-1] >= 0.999)

    @property
    def base_coverage(self) -> float:
        return self.history['base'][-1]

    def observe(self):
        """One Observation per robot (failed robots can only stay)."""
        self._observations = [build_observation(self.view(r.id), self.cfg) if r.alive
                              else dead_observation() for r in self.robots]
        return self._observations

    def step(self, actions) -> float:
        """Apply one option per robot, run one decision period, return the team reward."""
        assert self._observations is not None, 'call observe() before step()'
        for robot, action, obs in zip(self.robots, actions, self._observations):
            if robot.alive:
                self._apply(robot, int(action), obs)
        self._observations = None

        base_before = self._base_known()
        team_before = int((self.discovered & self.coverable).sum())
        for _ in range(self.cfg.ticks_per_decision):
            self.t += self.cfg.tick
            self._fail_robots()
            self._move()
            self._sense()
            self._communicate()
            self._record()
            if self.on_tick is not None:
                self.on_tick(self)
            if self.done:
                break
        gained_base = self._base_known() - base_before
        gained_team = int((self.discovered & self.coverable).sum()) - team_before
        m2 = self.cfg.cell ** 2
        return (gained_base + self.cfg.team_bonus * gained_team) * m2 * self.cfg.reward_per_m2

    def view(self, i: int) -> AgentView:
        """What robot i knows - the only input its policy gets."""
        r = self.robots[i]
        mates = [Teammate(id=j, pos=pos, goal=goal, age=self.t - heard)
                 for j, (pos, goal, heard) in r.teammates.items()]
        return AgentView(
            belief=r.belief, pos=r.pos, base=self.house.base, my_id=r.id, teammates=mates,
            beacons=sorted(r.known_beacons), beacons_left=r.beacons_left,
            unreported_cells=int(((r.belief == FREE) & ~r.reported).sum()),
            since_base_contact=self.t - r.last_base_contact,
            connected_to_base=r.connected_to_base,
            time_left=max(0.0, 1.0 - self.t / self.cfg.episode_seconds),
            fleet_size=self.cfg.n_robots, current_target=r.target)

    def global_state(self):
        """Ground-truth summary for the critic (training only, never seen by robots)."""
        cell = self.cfg.cell
        robots = np.zeros((MAX_ROBOTS, ROBOT_STATE_DIM), dtype=np.float32)
        present = np.zeros(MAX_ROBOTS, dtype=bool)
        for r in self.robots:
            present[r.id] = True
            unreported = ((r.belief == FREE) & ~r.reported).sum() * cell * cell
            robots[r.id] = [
                r.pos[1] * cell / DIST_SCALE, r.pos[0] * cell / DIST_SCALE,
                float(r.alive), float(r.connected_to_base), min(unreported / 50.0, 3.0),
                r.beacons_left / 2.0,
                math.hypot(r.pos[0] - self.house.base[0], r.pos[1] - self.house.base[1])
                * cell / DIST_SCALE,
                min((self.t - r.last_base_contact) / 60.0, 3.0)]
        scalars = np.array([
            self.history['base'][-1], self.history['team'][-1],
            max(0.0, 1.0 - self.t / self.cfg.episode_seconds),
            min(self.cfg.comm_range, 20.0) / 12.0, self.cfg.n_robots / MAX_ROBOTS,
            len(self.beacons) / 12.0, self.n_coverable * cell * cell / 300.0], dtype=np.float32)
        return robots, present, scalars

    # ------------------------------------------------------------ internals

    def _apply(self, robot: Robot, action: int, obs: Observation):
        kind, target = obs.targets[action]
        if not obs.mask[action]:   # invalid choice: stay put (policies shouldn't do this)
            kind, target = obs.targets[SLOT_STAY]
        if kind == 'drop':
            self.beacons.append(robot.pos)
            robot.known_beacons.add(robot.pos)
            robot.beacons_left -= 1
        if kind in ('stay', 'drop'):
            robot.target, robot.path = None, []
            return
        path = path_to(obs.pred, robot.pos, target, self.w)
        robot.target, robot.path = (target, path) if path is not None else (None, [])
        robot.progress = 0.0

    def _fail_robots(self):
        for r in self.robots:
            if r.alive and self.t >= r.fail_time:
                r.alive, r.path, r.target, r.connected_to_base = False, [], None, False

    def _move(self):
        for r in self.robots:
            budget = r.speed * self.cfg.tick if r.alive else 0.0
            while r.path and budget > 0.0:
                nr, nc = divmod(r.path[0], self.w)
                if not self.truth_trav[nr, nc]:
                    # Too close to an obstacle it hadn't seen yet: stop, replan next decision.
                    r.path, r.progress = [], 0.0
                    break
                diagonal = nr != r.pos[0] and nc != r.pos[1]
                need = self.cfg.cell * (math.sqrt(2.0) if diagonal else 1.0) - r.progress
                if budget >= need:
                    budget -= need
                    r.pos, r.progress = (nr, nc), 0.0
                    r.path.pop(0)
                else:
                    r.progress += budget
                    budget = 0.0

    def _sense(self):
        range_cells = self.cfg.lidar_range / self.cfg.cell
        for r in self.robots:
            if r.alive:
                free, hit = raycast(self.house.occupied, r.pos, self.cfg.lidar_rays, range_cells)
                r.belief[free] = FREE
                r.belief[hit] = OCCUPIED
                self.discovered[free] = True

    def _communicate(self):
        alive = [r for r in self.robots if r.alive]
        base_node = len(alive)
        nodes = [r.pos for r in alive] + [self.house.base] + self.beacons
        self.links = link_matrix(nodes, self.house.occupied, self.cfg.comm_range,
                                 self.cfg.wall_penalty, self.cfg.cell)
        for r in alive:
            r.connected_to_base = False
        for group in connected_groups(self.links):
            members = [alive[k] for k in group if k < base_node]
            has_base = base_node in group
            if not members or (len(members) == 1 and not has_base):
                continue
            maps = [m.belief for m in members] + ([self.base_belief] if has_base else [])
            merged = merge_maps(maps)
            beacons = set().union(*(m.known_beacons for m in members))
            if has_base:
                beacons |= self.base_beacons
                self.base_belief[:] = merged
                self.base_beacons = set(beacons)
            for m in members:
                m.belief[:] = merged
                m.known_beacons = set(beacons)
                for other in members:
                    if other is not m:
                        m.teammates[other.id] = (other.pos, other.target, self.t)
                if has_base:
                    m.connected_to_base = True
                    m.last_base_contact = self.t
                    m.reported = merged == FREE

    def _base_known(self) -> int:
        return int(((self.base_belief == FREE) & self.coverable).sum())

    def _record(self):
        self.history['t'].append(self.t)
        self.history['base'].append(self._base_known() / self.n_coverable)
        self.history['team'].append(int((self.discovered & self.coverable).sum())
                                    / self.n_coverable)
