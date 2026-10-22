#!/usr/bin/env bash
# Regenerate every result in the README from a finished training run:
#   1. pick the best checkpoint on validation houses -> fleet_rl/models/policy.npz
#   2. benchmark every policy on the test houses     -> docs/results/results.json
#   3. charts, the side-by-side GIF, the browser replays, the README tables
#
#   cd fleet_rl && ./make_results.sh [runs/main]      (PYTHON=/usr/bin/python3 to pick the interpreter)
set -euo pipefail
cd "$(dirname "$0")"
RUN=${1:-runs/main}
"${PYTHON:-python3}" -m fleet_rl.select_model "$RUN"/actor_0*.npz
"${PYTHON:-python3}" -m fleet_rl.evaluate
"${PYTHON:-python3}" -m fleet_rl.charts
"${PYTHON:-python3}" -m fleet_rl.media
"${PYTHON:-python3}" -m fleet_rl.replays
"${PYTHON:-python3}" -m fleet_rl.report
