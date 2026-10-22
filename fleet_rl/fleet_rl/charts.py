"""Charts for the README from docs/results/results.json (light and dark versions).

    python -m fleet_rl.charts [--results docs/results/results.json] [--progress runs/main/progress.csv]

Each policy keeps one color in every chart; "perfect comms" is a gray
reference, not a competitor.
"""

import argparse
import csv
import json
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

DOCS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'docs')

THEMES = {
    'light': dict(surface='#fcfcfb', text='#0b0b0b', muted='#52514e', grid='#e6e5e1',
                  reference='#8a8984', band='#f0efec',
                  series={'learned': '#2a78d6', 'return': '#eb6834', 'greedy': '#1baf7a',
                          'breadcrumb': '#eda100'}),
    'dark': dict(surface='#1a1a19', text='#ffffff', muted='#c3c2b7', grid='#33332f',
                 reference='#8f8e87', band='#26261f',
                 series={'learned': '#3987e5', 'return': '#d95926', 'greedy': '#199e70',
                         'breadcrumb': '#c98500'}),
}
LABELS = {'learned': 'learned (MAPPO)', 'return': 'return-to-base', 'greedy': 'greedy',
          'breadcrumb': 'breadcrumb beacons', 'perfect': 'perfect comms'}


def _style(ax, theme, title, subtitle, xlabel, ylabel):
    t = THEMES[theme]
    ax.figure.set_facecolor(t['surface'])
    ax.set_facecolor(t['surface'])
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(t['grid'])
        ax.spines[side].set_linewidth(1)
    ax.grid(True, color=t['grid'], linewidth=1, linestyle='-')
    ax.set_axisbelow(True)
    ax.tick_params(colors=t['muted'], labelsize=9, length=0)
    ax.set_xlabel(xlabel, color=t['muted'], fontsize=10)
    ax.set_ylabel(ylabel, color=t['muted'], fontsize=10)
    ax.set_title(title, loc='left', color=t['text'], fontsize=13, fontweight='semibold', pad=22)
    ax.text(0, 1.02, subtitle, transform=ax.transAxes, color=t['muted'], fontsize=9.5)


def _end_labels(ax, theme, points, x_text):
    """Direct labels at line ends; nudged apart if they'd collide, with leader lines."""
    t = THEMES[theme]
    lo, hi = ax.get_ylim()
    gap = (hi - lo) * 0.065
    points = sorted(points, key=lambda p: p[1])
    placed = []
    for _, y, _ in points:                       # push up until no two are closer than `gap`
        placed.append(max(y, placed[-1] + gap) if placed else y)
    shift = np.mean([y for _, y, _ in points]) - np.mean(placed)
    placed = [p + shift for p in placed]          # re-center the stack on the lines
    placed = [p - max(0.0, placed[-1] - (hi - gap * 0.3)) for p in placed]
    placed = [p + max(0.0, lo + gap * 0.3 - placed[0]) for p in placed]
    for (x, y, text), yt in zip(points, placed):
        if abs(yt - y) > gap * 0.15:
            ax.plot([x, x_text], [y, yt], color=t['grid'], linewidth=1, zorder=1, clip_on=False)
        ax.text(x_text, yt, ' ' + text, color=t['text'], fontsize=9, va='center')


def _line(ax, theme, x, y, policy, marker=False):
    t = THEMES[theme]
    color = t['series'].get(policy, t['reference'])
    style = dict(color=color, linewidth=2, solid_capstyle='round', solid_joinstyle='round',
                 label=LABELS[policy], zorder=3)
    if policy == 'perfect':
        style.update(linewidth=1.5, linestyle=(0, (4, 3)))
    if marker:
        style.update(marker='o', markersize=6, markeredgecolor=t['surface'], markeredgewidth=2)
    ax.plot(x, y, **style)


def _legend(ax, theme):
    t = THEMES[theme]
    leg = ax.legend(loc='lower right', frameon=False, fontsize=9, labelcolor=t['text'])
    return leg


def _save(fig, name, theme):
    folder = os.path.join(DOCS, 'img')
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f'{name}_{theme}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    return path


def coverage_over_time(results, theme):
    data = results['main']['3 robots, 5 m']
    fig, ax = plt.subplots(figsize=(8, 4.4))
    order = ['perfect', 'greedy', 'return', 'learned']
    labels = []
    for policy in order:
        curve = np.array(data[policy]['curve']) * 100
        x = np.arange(len(curve))
        _line(ax, theme, x, curve, policy)
        labels.append((x[-1], curve[-1], LABELS[policy]))
    ax.set_xlim(0, 240)
    ax.set_ylim(0, 102)
    ax.set_xticks(range(0, 241, 60))
    _style(ax, theme, 'How fast the base learns the house',
           '3 robots, 5 m radio range, 50 test houses never seen in training (mean)',
           'mission time (s)', '% of the house the base knows')
    _end_labels(ax, theme, labels, 244)
    return _save(fig, 'coverage_over_time', theme)


def comm_range(results, theme):
    data = results['range']
    ranges = [float(k.split()[0]) for k in data]
    fig, ax = plt.subplots(figsize=(8, 4.4))
    labels = []
    for policy in ['greedy', 'return', 'learned']:
        y = [data[k][policy]['mean'][0] * 100 for k in data]
        _line(ax, theme, ranges, y, policy, marker=True)
        labels.append((ranges[-1], y[-1], LABELS[policy]))
    perfect = results['main']['3 robots, 5 m']['perfect']['mean'][0] * 100
    ax.axhline(perfect, color=THEMES[theme]['reference'], linewidth=1.5, linestyle=(0, (4, 3)),
               label=LABELS['perfect'], zorder=2)
    labels.append((ranges[-1], perfect, 'perfect comms'))
    ax.set_xlim(1.5, 12.5)
    ax.set_ylim(0, 100)
    ax.set_xticks(ranges)
    _style(ax, theme, 'Ahead of the best heuristic at every radio range',
           'mean % the base knows over the mission; 3 robots, 30 test houses per range',
           'radio range in open space (m); each wall costs 3 m', 'mean coverage at base (%)')
    _end_labels(ax, theme, labels, 12.7)
    return _save(fig, 'comm_range', theme)


def fleet_size(results, theme):
    data = results['fleet']
    sizes = [int(k.split()[0]) for k in data]
    fig, ax = plt.subplots(figsize=(8, 4.4))
    t = THEMES[theme]
    ax.axvspan(1.7, 4.3, color=t['band'], zorder=0, linewidth=0)
    ax.text(3, 3, 'trained on 2–4 robots', ha='center', color=t['muted'], fontsize=9)
    labels = []
    for policy in ['perfect', 'greedy', 'return', 'learned']:
        y = [data[k][policy]['mean'][0] * 100 for k in data]
        _line(ax, theme, sizes, y, policy, marker=policy != 'perfect')
        labels.append((sizes[-1], y[-1], LABELS[policy]))
    ax.set_xlim(1.7, 6.3)
    ax.set_ylim(0, 100)
    ax.set_xticks(sizes)
    _style(ax, theme, 'One model, any fleet size',
           'mean % the base knows over the mission; 5 m range, 30 test houses per size',
           'robots in the fleet', 'mean coverage at base (%)')
    _end_labels(ax, theme, labels, 6.4)
    return _save(fig, 'fleet_size', theme)


def _training_rows(stages):
    """Concatenate training stages: [(progress.csv, last update to keep or None), ...]."""
    steps, cov, boundaries, offset = [], [], [], 0.0
    for path, last in stages:
        with open(path) as f:
            rows = [r for r in csv.DictReader(f)
                    if r['mean_coverage'] and (last is None or int(r['update']) <= last)]
        steps += [offset + float(r['env_steps']) for r in rows]
        cov += [float(r['mean_coverage']) for r in rows]
        offset = steps[-1]
        boundaries.append(offset)
    return np.array(steps) / 1e6, np.array(cov) * 100, [b / 1e6 for b in boundaries[:-1]]


def learning_curve(results, stages, theme):
    steps, cov, boundaries = _training_rows(stages)
    window = 25
    smooth = np.convolve(cov, np.ones(window) / window, mode='valid')
    fig, ax = plt.subplots(figsize=(8, 4.4))
    t = THEMES[theme]
    ax.plot(steps, cov, color=t['series']['learned'], alpha=0.18, linewidth=1)
    _line(ax, theme, steps[window - 1:], smooth, 'learned')
    for b in boundaries:
        ax.axvline(b, color=t['reference'], linewidth=1)
        ax.text(b, 4, '  fine-tuned from here\n  (goal stand-off added)', color=t['muted'],
                fontsize=8.5, va='bottom')
    labels = [(steps[-1], smooth[-1], LABELS['learned'])]
    mix = results.get('mix', {}).get('training mix', {})
    for policy in ('greedy', 'return', 'breadcrumb'):
        if policy in mix:
            y = mix[policy]['mean'][0] * 100
            ax.axhline(y, color=t['series'][policy], linewidth=1.5, linestyle=(0, (4, 3)),
                       label=LABELS[policy], zorder=2)
            labels.append((steps[-1], y, LABELS[policy]))
    ax.set_xlim(0, steps[-1])
    ax.set_ylim(0, 100)
    _style(ax, theme, 'Training on a laptop CPU',
           'training episodes (random fleet size, range, beacons, failures); '
           'dashed = baselines on the same mix',
           'decisions simulated (millions)', 'mean coverage at base (%)')
    _end_labels(ax, theme, labels, steps[-1] * 1.015)
    return _save(fig, 'learning_curve', theme)


RUNS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'runs')
# Stage 1 trained from scratch; stage 2 resumed from its update-400 checkpoint.
TRAINING_STAGES = [(os.path.join(RUNS, 'v1_no_standoff', 'progress.csv'), 400),
                   (os.path.join(RUNS, 'main', 'progress.csv'), None)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--results', default=os.path.join(DOCS, 'results', 'results.json'))
    args = p.parse_args()
    with open(args.results) as f:
        results = json.load(f)
    for theme in THEMES:
        print(coverage_over_time(results, theme))
        print(comm_range(results, theme))
        print(fleet_size(results, theme))
        if all(os.path.exists(path) for path, _ in TRAINING_STAGES):
            print(learning_curve(results, TRAINING_STAGES, theme))


if __name__ == '__main__':
    main()
