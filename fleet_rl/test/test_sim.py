"""Houses, the simulator, observations and the hand-written baselines."""

import math
import os

import numpy as np
import pytest

from fleet_rl.baselines import Breadcrumb, Greedy, ReturnToBase
from fleet_rl.config import MAX_FRONTIERS, SimConfig
from fleet_rl.grid import traversable
from fleet_rl.houses import generate_house, load_sdf_house
from fleet_rl.observations import (MAX_OPTIONS, OPTION_DIM, SELF_DIM, SLOT_BASE, SLOT_DROP,
                                   SLOT_STAY, F_PATH)
from fleet_rl.sim import FleetSim

WORLD = os.path.join(os.path.dirname(__file__), '..', '..', 'rover_bringup', 'worlds',
                     'rover_world.sdf')


def run(sim, policy):
    rewards = []
    while not sim.done:
        rewards.append(sim.step([policy.act(o) for o in sim.observe()]))
    return rewards


def test_houses_are_deterministic_and_valid():
    cfg = SimConfig()
    assert (generate_house(7).occupied == generate_house(7).occupied).all()
    for seed in range(15):
        house = generate_house(seed)
        occ = house.occupied
        assert occ[0].all() and occ[-1].all() and occ[:, 0].all() and occ[:, -1].all()
        trav = traversable(occ, cfg.radius_cells)
        assert trav[house.base]
        assert all(trav[s] for s in house.spawns)


def test_gazebo_house_loads():
    house = load_sdf_house(WORLD, base_xy=(0.0, 0.0))
    h, w = house.shape
    assert 16.0 <= w * house.cell <= 19.0 and 12.0 <= h * house.cell <= 15.0
    assert not house.occupied[house.base]
    # The north wall's center (x=3, y=7.5) is solid; the living room center isn't.
    assert house.occupied[house.to_cell(3.0, 7.5)]
    assert house.to_world(house.to_cell(1.03, -2.0)) == pytest.approx((1.1, -1.9))


def test_episode_bookkeeping():
    house = generate_house(3)
    cfg = SimConfig()
    sim = FleetSim(house, cfg, seed=0)
    base0, team0 = sim.history['base'][0], sim.history['team'][0]
    rewards = run(sim, Greedy())
    base, team = np.array(sim.history['base']), np.array(sim.history['team'])
    assert (np.diff(base) >= 0).all() and (np.diff(team) >= 0).all()
    assert (base <= team + 1e-12).all()
    # Rewards add up to exactly the coverage gained (in m^2, scaled).
    m2 = sim.n_coverable * cfg.cell ** 2 * cfg.reward_per_m2
    expected = (base[-1] - base0) * m2 + cfg.team_bonus * (team[-1] - team0) * m2
    assert math.isclose(sum(rewards), expected, rel_tol=1e-9)
    assert sim.t <= cfg.episode_seconds + 1e-9


def test_perfect_comms_base_knows_everything_seen():
    sim = FleetSim(generate_house(4), SimConfig(comm_range=math.inf), seed=0)
    run(sim, Greedy())
    assert sim.history['base'] == sim.history['team']


def test_out_of_range_robots_keep_separate_maps():
    sim = FleetSim(generate_house(5), SimConfig(comm_range=0.0), seed=0)
    for _ in range(10):
        sim.step([Greedy().act(o) for o in sim.observe()])
    a, b = sim.robots[0].belief, sim.robots[1].belief
    assert (a != b).any()
    assert not sim.robots[0].connected_to_base or sim.robots[0].pos == sim.house.base


def test_dropout_robot_stops():
    sim = FleetSim(generate_house(6), SimConfig(dropout_prob=1.0), seed=1)
    dead = next(r for r in sim.robots if r.fail_time < math.inf)
    run(sim, Greedy())
    assert not dead.alive
    obs = sim.observe()[dead.id]
    assert obs.mask.tolist() == [i == SLOT_STAY for i in range(MAX_OPTIONS)]


def test_observation_shapes_and_masks():
    sim = FleetSim(generate_house(8), SimConfig(beacons_per_robot=1), seed=0)
    obs = sim.observe()[0]
    assert obs.options.shape == (MAX_OPTIONS, OPTION_DIM)
    assert obs.self_features.shape == (SELF_DIM,)
    assert obs.mask[SLOT_STAY] and obs.mask[SLOT_BASE] and obs.mask[SLOT_DROP]
    paths = obs.options[:MAX_FRONTIERS, F_PATH][obs.mask[:MAX_FRONTIERS]]
    assert (np.diff(paths) >= 0).all()     # frontiers nearest first
    assert not obs.options[~obs.mask].any()


def test_beacon_drop():
    sim = FleetSim(generate_house(9), SimConfig(beacons_per_robot=1), seed=0)
    sim.observe()
    pos = sim.robots[0].pos
    sim.step([SLOT_DROP, SLOT_STAY, SLOT_STAY])
    assert sim.beacons == [pos] and sim.robots[0].beacons_left == 0
    assert not sim.observe()[0].mask[SLOT_DROP]


def test_baselines_behave():
    obs = FleetSim(generate_house(10), SimConfig(), seed=0).observe()[0]
    assert obs.mask[Greedy().act(obs)]
    # Overdue and disconnected -> return goes home.
    obs.self_features[1] = 2.0             # 120 s since base contact
    obs.self_features[2] = 0.0             # not connected
    assert ReturnToBase(period_s=60).act(obs) == SLOT_BASE
    # Connected, at the edge of range, beacons left -> breadcrumb drops one.
    obs.self_features[2] = 1.0
    obs.self_features[6] = 0.1
    obs.mask[SLOT_DROP] = True
    assert Breadcrumb().act(obs) == SLOT_DROP


def test_communication_beats_none():
    """Sanity check of the whole premise: perfect comms keeps the base better informed."""
    house = generate_house(11)
    limited = FleetSim(house, SimConfig(comm_range=3.0), seed=0)
    perfect = FleetSim(house, SimConfig(comm_range=math.inf), seed=0)
    run(limited, Greedy())
    run(perfect, Greedy())
    assert np.mean(perfect.history['base']) > np.mean(limited.history['base'])


def test_teammate_and_base_goals_stop_short():
    from fleet_rl.config import FREE
    from fleet_rl.observations import AgentView, SLOT_TEAMMATE, Teammate, build_observation

    belief = np.full((20, 60), FREE, dtype=np.int8)
    belief[0, :] = belief[-1, :] = belief[:, 0] = belief[:, -1] = 1
    view = AgentView(belief=belief, pos=(10, 5), base=(10, 50), my_id=0,
                     teammates=[Teammate(id=1, pos=(10, 30))], fleet_size=2)
    obs = build_observation(view, SimConfig())
    (_, mate_goal), (_, base_goal) = obs.targets[SLOT_TEAMMATE], obs.targets[SLOT_BASE]
    assert mate_goal[0] == 10 and 30 - mate_goal[1] == pytest.approx(1.5 / 0.2, abs=1)
    assert base_goal[0] == 10 and 50 - base_goal[1] == pytest.approx(1.0 / 0.2, abs=1)
    # Already close to the base: the goal is where the robot stands.
    view.pos = (10, 47)
    assert build_observation(view, SimConfig()).targets[SLOT_BASE][1] == (10, 47)
