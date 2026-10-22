"""README media: a side-by-side GIF of return-to-base vs learned on a typical test house.

    python -m fleet_rl.media      # needs docs/results/results.json from fleet_rl.evaluate

"Typical" = the test house whose learned-minus-baseline difference is closest
to the average difference, so the GIF shows the usual case, not the best one.
"""

import argparse
import json
import os

import numpy as np

from .config import SimConfig
from .evaluate import DOCS, TEST_SEED, make_policy
from .houses import generate_house
from .policy import DEFAULT_MODEL
from .render import Recorder, save_side_by_side
from .sim import FleetSim


def record(house, cfg, policy, seed):
    sim = FleetSim(house, cfg, seed=seed)
    recorder = Recorder(every=2)
    sim.on_tick = recorder
    while not sim.done:
        sim.step([policy.act(o) for o in sim.observe()])
    return recorder.frames, float(np.mean(sim.history['base']))


def typical_test_house(results):
    """Index of the main-benchmark test house whose learned-minus-return gap is closest to average."""
    runs = results['main']['3 robots, 5 m']
    diff = np.array(runs['learned']['per_episode_mean']) - np.array(runs['return']['per_episode_mean'])
    return int(np.argmin(np.abs(diff - diff.mean()))), float(diff.mean())


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--model', default=DEFAULT_MODEL)
    p.add_argument('--results', default=os.path.join(DOCS, 'results', 'results.json'))
    p.add_argument('--out', default=os.path.join(DOCS, 'img', 'side_by_side.gif'))
    args = p.parse_args()
    with open(args.results) as f:
        results = json.load(f)
    period = results.get('return_period_used', 60.0)
    i, mean_diff = typical_test_house(results)
    house = generate_house(TEST_SEED + i)
    cfg = SimConfig()
    frames_a, mean_a = record(house, cfg, make_policy('return', args.model, period), i)
    frames_b, mean_b = record(house, cfg, make_policy('learned', args.model, period), i)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    save_side_by_side(house, frames_a, frames_b, ('return-to-base', 'learned'), args.out)
    print(f'test house {TEST_SEED + i}: return {mean_a:.3f} vs learned {mean_b:.3f} '
          f'(average difference {mean_diff:+.3f}) -> {os.path.abspath(args.out)}')


if __name__ == '__main__':
    main()
