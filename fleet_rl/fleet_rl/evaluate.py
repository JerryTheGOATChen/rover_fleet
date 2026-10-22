"""Benchmark every policy on the same held-out houses.

    python -m fleet_rl.evaluate                       # all experiments -> docs/results/results.json
    python -m fleet_rl.evaluate --only main gazebo    # a subset

Every policy plays exactly the same houses with the same seeds, so results
are paired: "learned minus best baseline" is measured house by house.
Test houses use seeds from 1,000,000 up; training only ever saw seeds below
100,000, and tuning the return-to-base period uses 500,000+.

Metric: mean coverage = the fraction of the house the base knows, averaged
over the whole mission (higher = the operator learns the house sooner).
Also reported: coverage at the deadline, and time until the base knows 90%.
"""

import argparse
import dataclasses
import json
import math
import os
from functools import partial
from multiprocessing import Pool

import numpy as np

from .baselines import Breadcrumb, Greedy, ReturnToBase
from .config import SimConfig, sample_training_config
from .houses import GAZEBO_BASE, GAZEBO_SPAWNS, gazebo_world_path, generate_house, load_sdf_house
from .policy import DEFAULT_MODEL, LearnedPolicy
from .sim import FleetSim

TEST_SEED = 1_000_000
DOCS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'docs')
TUNE_SEED = 500_000


def make_policy(name, model, period):
    if name in ('greedy', 'perfect'):
        return Greedy()
    if name == 'return':
        return ReturnToBase(period_s=period)
    if name == 'breadcrumb':
        return Breadcrumb(period_s=period)
    if name == 'learned':
        return LearnedPolicy(model)
    raise ValueError(name)


def run_episode(task, model=DEFAULT_MODEL, period=60.0):
    """One episode. task = (policy, house seed or 'gazebo', sim seed, SimConfig kwargs)."""
    policy_name, house_seed, sim_seed, overrides = task
    cfg = SimConfig(**overrides)
    if policy_name == 'perfect':
        cfg = SimConfig(**{**overrides, 'comm_range': math.inf})
    if house_seed == 'gazebo':
        house = load_sdf_house(gazebo_world_path(), base_xy=GAZEBO_BASE,
                               spawn_xys=GAZEBO_SPAWNS)
    else:
        house = generate_house(house_seed, sim=cfg)
    sim = FleetSim(house, cfg, seed=sim_seed)
    policy = make_policy(policy_name, model, period)
    while not sim.done:
        sim.step([policy.act(o) for o in sim.observe()])
    # Pad to the full mission if it ended early (everything already known).
    n_ticks = round(cfg.episode_seconds / cfg.tick) + 1
    base = sim.history['base'] + [sim.history['base'][-1]] * (n_ticks - len(sim.history['base']))
    base = np.array(base[:n_ticks])
    reached = np.flatnonzero(base >= 0.9)
    return dict(mean=float(base.mean()), final=float(base[-1]),
                t90=float(reached[0] * cfg.tick) if len(reached) else None,
                curve=base[::2].round(4).tolist(),      # one point per second
                beacons=len(sim.beacons))


def summarize(results):
    """Mean and 95% confidence interval of each metric over episodes."""
    out = {}
    for key in ('mean', 'final', 'beacons'):
        values = np.array([r[key] for r in results], dtype=float)
        half = 1.96 * values.std(ddof=1) / math.sqrt(len(values)) if len(values) > 1 else 0.0
        out[key] = [float(values.mean()), float(half)]
    t90 = [r['t90'] for r in results]
    out['t90_reached'] = float(np.mean([t is not None for t in t90]))
    out['t90_median'] = float(np.median([t if t is not None else math.inf for t in t90]))
    out['curve'] = np.mean([r['curve'] for r in results], axis=0).round(4).tolist()
    out['per_episode_mean'] = [round(r['mean'], 4) for r in results]
    return out


EXPERIMENTS = {
    # name: (policies, list of (label, SimConfig overrides), number of houses)
    'main': (['greedy', 'return', 'learned', 'perfect'], [('3 robots, 5 m', {})], 50),
    'range': (['greedy', 'return', 'learned'],
              [(f'{r} m', {'comm_range': float(r)}) for r in (2, 3, 4, 5, 6, 8, 10, 12)], 30),
    'fleet': (['greedy', 'return', 'learned', 'perfect'],
              [(f'{n} robots', {'n_robots': n}) for n in (2, 3, 4, 5, 6)], 30),
    'beacons': (['return', 'breadcrumb', 'learned'],
                [('2 beacons each', {'beacons_per_robot': 2})], 50),
    'dropout': (['greedy', 'return', 'learned'], [('one robot fails', {'dropout_prob': 1.0})], 50),
    'gazebo': (['greedy', 'return', 'learned', 'perfect'],
               [('Gazebo house', {'speed_jitter': 0.25})], 20),
    # Each episode drawn like a training episode (random fleet, range, beacons, failures).
    'mix': (['greedy', 'return', 'breadcrumb', 'learned'], [('training mix', None)], 100),
}


def tune_return_period(pool, periods=(30, 45, 60, 90, 120), n_houses=30):
    """Pick return-to-base's period on tuning houses (not the test houses)."""
    scores = {}
    for period in periods:
        tasks = [('return', TUNE_SEED + i, i, {}) for i in range(n_houses)]
        res = pool.map(partial(run_episode, period=period), tasks)
        scores[period] = float(np.mean([r['mean'] for r in res]))
        print(f'  return period {period:4d} s: mean coverage {scores[period]:.3f}', flush=True)
    return max(scores, key=scores.get), scores


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--model', default=DEFAULT_MODEL)
    p.add_argument('--out', default=os.path.join(DOCS, 'results', 'results.json'))
    p.add_argument('--only', nargs='*', default=None, help='experiment names to run')
    p.add_argument('--workers', type=int, default=os.cpu_count())
    p.add_argument('--period', type=float, default=None,
                   help='return-to-base period (default: tune it first)')
    args = p.parse_args()

    results = {}
    if os.path.exists(args.out):
        with open(args.out) as f:
            results = json.load(f)
    with Pool(args.workers) as pool:
        period = args.period
        if period is None:
            print('tuning return-to-base period...', flush=True)
            period, scores = tune_return_period(pool)
            results['return_period'] = {'best': period, 'scores': scores}
        results['return_period_used'] = period
        for name, (policies, conditions, n_houses) in EXPERIMENTS.items():
            if args.only and name not in args.only:
                continue
            results[name] = {}
            for label, overrides in conditions:
                results[name][label] = {}
                for policy in policies:
                    if name == 'gazebo':
                        tasks = [(policy, 'gazebo', i, overrides) for i in range(n_houses)]
                    elif overrides is None:   # training mix: a random config per episode
                        tasks = [(policy, TEST_SEED + 100_000 + i, i, dataclasses.asdict(
                            sample_training_config(np.random.default_rng(i))))
                            for i in range(n_houses)]
                    else:
                        tasks = [(policy, TEST_SEED + i, i, overrides) for i in range(n_houses)]
                    res = pool.map(partial(run_episode, model=args.model, period=period), tasks)
                    results[name][label][policy] = summarize(res)
                    m = results[name][label][policy]['mean']
                    print(f'{name:8s} {label:16s} {policy:10s} mean coverage '
                          f'{m[0]:.3f} ± {m[1]:.3f}', flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as f:
        json.dump(results, f)
    print('wrote', os.path.abspath(args.out))


if __name__ == '__main__':
    main()
