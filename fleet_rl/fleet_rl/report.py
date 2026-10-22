"""Markdown tables for the README, straight from docs/results/results.json.

    python -m fleet_rl.report

Differences are paired: every policy played the same houses with the same
seeds, so "learned - baseline" is computed house by house, with a 95%
confidence interval (normal approximation) over houses.
"""

import argparse
import json
import math
import os

import numpy as np

from .evaluate import DOCS

NAMES = {'learned': 'Learned (MAPPO)', 'return': 'Return-to-base', 'greedy': 'Greedy',
         'breadcrumb': 'Breadcrumb beacons', 'perfect': 'Perfect comms (reference)'}


def pct(x):
    return f'{100 * x:.1f}%'


def paired(a, b):
    """Mean and 95% CI half-width of a - b over episodes, in percentage points."""
    d = (np.array(a) - np.array(b)) * 100
    return d.mean(), 1.96 * d.std(ddof=1) / math.sqrt(len(d))


def best_baseline(cond, candidates=('return', 'greedy', 'breadcrumb')):
    present = [p for p in candidates if p in cond]
    return max(present, key=lambda p: cond[p]['mean'][0])


def main_table(results):
    cond = results['main']['3 robots, 5 m']
    rows = ['| Strategy | Mean coverage at base | At the 240 s deadline | Time to 90% (median) |',
            '|---|---|---|---|']
    for p in ('learned', 'return', 'greedy', 'perfect'):
        c = cond[p]
        t90 = f"{c['t90_median']:.0f} s" if math.isfinite(c['t90_median']) else 'not reached'
        rows.append(f"| {NAMES[p]} | {pct(c['mean'][0])} ± {100 * c['mean'][1]:.1f} | "
                    f"{pct(c['final'][0])} | {t90} |")
    return '\n'.join(rows)


def robustness_table(results):
    rows = ['| Setting | Learned | Best baseline | Difference (paired, 95% CI) |', '|---|---|---|---|']

    def add(label, cond, baselines=('return', 'greedy', 'breadcrumb')):
        base = best_baseline(cond, baselines)
        d, ci = paired(cond['learned']['per_episode_mean'], cond[base]['per_episode_mean'])
        rows.append(f"| {label} | {pct(cond['learned']['mean'][0])} | "
                    f"{pct(cond[base]['mean'][0])} ({NAMES[base].lower()}) | "
                    f"{d:+.1f} ± {ci:.1f} pts |")

    add('3 robots, 5 m radio (main)', results['main']['3 robots, 5 m'])
    for label in ('2 m', '3 m', '8 m', '12 m'):
        add(f'Radio range {label}', results['range'][label])
    for label in ('2 robots', '5 robots', '6 robots'):
        note = ' (never trained on)' if label in ('5 robots', '6 robots') else ''
        add(f'{label.capitalize()}{note}', results['fleet'][label])
    add('2 relay beacons each', results['beacons']['2 beacons each'])
    add('One robot fails mid-mission', results['dropout']['one robot fails'])
    add('Random settings, like training', results['mix']['training mix'])
    add('Gazebo house (grid sim)', results['gazebo']['Gazebo house'])
    return '\n'.join(rows)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--results', default=os.path.join(DOCS, 'results', 'results.json'))
    args = p.parse_args()
    with open(args.results) as f:
        results = json.load(f)
    print(main_table(results))
    print()
    print(robustness_table(results))
    print()
    print(f"Return-to-base period: {results['return_period_used']:.0f} s "
          f"(tuned on separate houses: {results['return_period']['scores']})")


if __name__ == '__main__':
    main()
