"""Pick the training checkpoint with the best validation score and ship it.

    python -m fleet_rl.select_model runs/main/actor_0*.npz

Every checkpoint plays the same 60 validation episodes, drawn like training
episodes (random fleet size, radio range, beacons, failures) on houses from
the validation seeds (500,000+). The test houses (1,000,000+) are only used
afterwards, by fleet_rl.evaluate, so the reported results are not tuned on.
Writes fleet_rl/models/policy.npz plus policy.json saying where it came from.
"""

import argparse
import dataclasses
import json
import os
import shutil
from functools import partial
from multiprocessing import Pool

import numpy as np

from .config import sample_training_config
from .evaluate import TUNE_SEED, run_episode
from .policy import DEFAULT_MODEL

N_EPISODES = 60


def validation_tasks():
    return [('learned', TUNE_SEED + 1000 + i, i,
             dataclasses.asdict(sample_training_config(np.random.default_rng(10_000 + i))))
            for i in range(N_EPISODES)]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('checkpoints', nargs='+')
    p.add_argument('--out', default=DEFAULT_MODEL)
    p.add_argument('--workers', type=int, default=os.cpu_count())
    args = p.parse_args()

    scores = {}
    with Pool(args.workers) as pool:
        for path in args.checkpoints:
            results = pool.map(partial(run_episode, model=path), validation_tasks())
            scores[path] = float(np.mean([r['mean'] for r in results]))
            print(f'{os.path.basename(path):24s} validation mean coverage {scores[path]:.4f}',
                  flush=True)
    best = max(scores, key=scores.get)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    shutil.copyfile(best, args.out)
    with open(os.path.splitext(args.out)[0] + '.json', 'w') as f:
        json.dump({'checkpoint': os.path.basename(best), 'validation_mean_coverage': scores[best],
                   'validation_episodes': N_EPISODES,
                   'all_scores': {os.path.basename(k): v for k, v in scores.items()}}, f, indent=2)
    print(f'best: {best} ({scores[best]:.4f}) -> {os.path.abspath(args.out)}')


if __name__ == '__main__':
    main()
