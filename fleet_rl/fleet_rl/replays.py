"""Record episodes for the browser replay demo (docs/demo/index.html).

    python -m fleet_rl.replays            # writes docs/demo/replays.js

Every combination of house x radio range x policy is simulated with the same
seed, so the page can play any two policies side by side. Maps only ever gain
information, so each frame stores just the cells that became known since the
previous frame.
"""

import argparse
import json
import os

import numpy as np

from .config import FREE, OCCUPIED, SimConfig
from .evaluate import DOCS, TEST_SEED, make_policy
from .houses import GAZEBO_BASE, GAZEBO_SPAWNS, gazebo_world_path, generate_house, load_sdf_house
from .media import typical_test_house
from .policy import DEFAULT_MODEL
from .sim import FleetSim

# 'generated' becomes the typical test house (see media.typical_test_house) once results exist.
HOUSES = {'generated': 1_000_000, 'gazebo': 'gazebo'}
RANGES = (3.0, 5.0, 8.0)
POLICIES = ('learned', 'return', 'greedy')


def load_house(key):
    if HOUSES[key] == 'gazebo':
        return load_sdf_house(gazebo_world_path(), base_xy=GAZEBO_BASE, spawn_xys=GAZEBO_SPAWNS)
    return generate_house(HOUSES[key])


def record(house, cfg, policy, seed=0, every=2):
    """Simulate one episode; return its frames as deltas."""
    sim = FleetSim(house, cfg, seed=seed)
    n = len(sim.robots)
    known_base = np.zeros(house.shape, dtype=bool)
    known_robot = [np.zeros(house.shape, dtype=bool) for _ in range(n)]
    frames = []

    def snap(s):
        base_known = s.base_belief != -1
        new_base = np.flatnonzero(base_known & ~known_base)
        known_base[base_known] = True
        robot_new = []
        for i, r in enumerate(s.robots):
            free = r.belief == FREE
            robot_new.append(np.flatnonzero(free & ~known_robot[i]).tolist())
            known_robot[i] |= free
        alive = [r for r in s.robots if r.alive]
        nodes = [r.pos for r in alive] + [house.base] + list(s.beacons)
        links = [[i, j] for i in range(len(nodes)) for j in range(i + 1, len(nodes))
                 if s.links[i, j]]
        frames.append({
            't': round(s.t, 1),
            'cov': round(s.base_coverage, 4),
            'robots': [[r.pos[0], r.pos[1], int(r.alive),
                        *(r.target if r.target is not None else (-1, -1))] for r in s.robots],
            'nodes': [list(map(int, p)) for p in nodes],
            'links': links,
            'beacons': [list(map(int, b)) for b in s.beacons],
            'base_free': [int(c) for c in new_base if s.base_belief.flat[c] == FREE],
            'base_occ': [int(c) for c in new_base if s.base_belief.flat[c] == OCCUPIED],
            'robot_free': robot_new,
        })

    ticks = [0]

    def on_tick(s):
        ticks[0] += 1
        if ticks[0] % every == 0 or s.done:
            snap(s)

    snap(sim)
    sim.on_tick = on_tick
    while not sim.done:
        sim.step([policy.act(o) for o in sim.observe()])
    return frames


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--model', default=DEFAULT_MODEL)
    p.add_argument('--out', default=os.path.join(DOCS, 'demo', 'replays.js'))
    args = p.parse_args()
    period = 60.0
    results_path = os.path.join(DOCS, 'results', 'results.json')
    if os.path.exists(results_path):   # tuned return-to-base period + the typical test house
        with open(results_path) as f:
            results = json.load(f)
        period = results.get('return_period_used', period)
        HOUSES['generated'] = TEST_SEED + typical_test_house(results)[0]
    data = {'houses': {}, 'episodes': {}, 'ranges': list(RANGES), 'policies': list(POLICIES),
            'return_period': period}
    for key in HOUSES:
        house = load_house(key)
        h, w = house.shape
        data['houses'][key] = {'h': h, 'w': w, 'cell': house.cell, 'base': list(house.base),
                               'walls': np.flatnonzero(house.occupied.ravel()).tolist()}
        for comm in RANGES:
            cfg = SimConfig(comm_range=comm)
            for name in POLICIES:
                frames = record(house, cfg, make_policy(name, args.model, period))
                data['episodes'][f'{key}|{comm:g}|{name}'] = frames
                print(f'{key:9s} {comm:4.1f} m {name:8s} final base coverage '
                      f'{frames[-1]["cov"]:.2f}  mean {np.mean([f["cov"] for f in frames]):.3f}',
                      flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as f:
        f.write('window.REPLAYS = ')
        json.dump(data, f, separators=(',', ':'))
        f.write(';\n')
    print('wrote', os.path.abspath(args.out), f'({os.path.getsize(args.out) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
