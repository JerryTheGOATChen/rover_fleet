"""The radio model.

Two nodes (robots, the base, beacons) can talk if

    distance + wall_penalty * obstacles_crossed <= comm_range

which is a simple path-loss model: distance and every wall weaken the
signal. Links chain: nodes in the same connected group all share data, so a
robot out of the base's range can still report through a teammate or a
beacon in between.
"""

import math

import numpy as np

from .config import FREE, OCCUPIED, UNKNOWN
from .grid import walls_between


def link_matrix(points, occupied, comm_range, wall_penalty, cell):
    """Symmetric bool matrix: adj[i, j] is True if nodes i and j can talk."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    n = len(points)
    adj = np.eye(n, dtype=bool)
    if n < 2:
        return adj
    if math.isinf(comm_range):
        adj[:] = True
        return adj
    i, j = np.triu_indices(n, k=1)
    dist = np.hypot(*(points[i] - points[j]).T) * cell
    close = dist <= comm_range
    i, j, dist = i[close], j[close], dist[close]
    effective = dist + wall_penalty * walls_between(occupied, points[i], points[j])
    ok = effective <= comm_range
    adj[i[ok], j[ok]] = adj[j[ok], i[ok]] = True
    return adj


def connected_groups(adj):
    """Connected components of the link graph, as lists of node indices."""
    n = len(adj)
    group_of = [-1] * n
    groups = []
    for start in range(n):
        if group_of[start] >= 0:
            continue
        group, todo = [], [start]
        group_of[start] = len(groups)
        while todo:
            node = todo.pop()
            group.append(node)
            for other in np.flatnonzero(adj[node]):
                if group_of[other] < 0:
                    group_of[other] = len(groups)
                    todo.append(int(other))
        groups.append(sorted(group))
    return groups


def merge_maps(maps):
    """Combine belief maps: occupied if anyone saw an obstacle, else free if anyone saw free."""
    stack = np.stack(maps)
    merged = np.full(stack.shape[1:], UNKNOWN, dtype=np.int8)
    merged[(stack == FREE).any(axis=0)] = FREE
    merged[(stack == OCCUPIED).any(axis=0)] = OCCUPIED
    return merged


def signal_margin(blocked, points, fixed_nodes, comm_range, wall_penalty, cell):
    """How much radio range is left at each point, to the best fixed node (base or beacon).

    1 = right next to it, 0 = at the edge of range, negative = out of range
    (clipped to -1). Uses only the walls in `blocked`, i.e. what the robot
    knows about, so it is an estimate.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    fixed = np.asarray(fixed_nodes, dtype=float).reshape(-1, 2)
    if math.isinf(comm_range):
        return np.ones(len(points))
    if len(points) == 0 or len(fixed) == 0:
        return np.full(len(points), -1.0)
    a = np.repeat(points, len(fixed), axis=0)
    b = np.tile(fixed, (len(points), 1))
    effective = np.hypot(*(a - b).T) * cell + wall_penalty * walls_between(blocked, a, b)
    best = effective.reshape(len(points), len(fixed)).min(axis=1)
    return np.clip((comm_range - best) / max(comm_range, 1e-6), -1.0, 1.0)
