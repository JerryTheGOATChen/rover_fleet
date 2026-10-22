"""Grid geometry: lidar raycasting, counting walls on a line, clearance, shortest paths.

Grids are numpy arrays indexed [row, col]. Row is +y and col is +x, the same
layout as nav_msgs/OccupancyGrid, so a ROS map converts without flipping.
Positions are (row, col) cells; a cell's center is at (row + 0.5, col + 0.5).
"""

import math
from functools import lru_cache

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra


@lru_cache(maxsize=8)
def _ray_samples(n_rays: int, range_cells: float):
    """Offsets (in cells) of points every half cell along each lidar ray."""
    angles = np.linspace(0.0, 2.0 * np.pi, n_rays, endpoint=False)
    steps = np.arange(1, int(range_cells * 2) + 1) * 0.5
    return np.sin(angles)[:, None] * steps, np.cos(angles)[:, None] * steps


def raycast(occupied: np.ndarray, pos, n_rays: int, range_cells: float):
    """What a 360-degree lidar at cell `pos` sees.

    Each ray stops at the first occupied cell or the map edge. Returns
    (free_cells, hit_cells), each a (rows, cols) pair of index arrays.
    Sampling every half cell can't skip a wall at least one cell thick.
    """
    h, w = occupied.shape
    dr, dc = _ray_samples(n_rays, float(range_cells))
    rows = np.floor(pos[0] + 0.5 + dr).astype(np.intp)
    cols = np.floor(pos[1] + 0.5 + dc).astype(np.intp)
    inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
    blocked = ~inside
    blocked[inside] = occupied[rows[inside], cols[inside]]

    n_steps = blocked.shape[1]
    first = np.where(blocked.any(axis=1), blocked.argmax(axis=1), n_steps)
    visible = np.arange(n_steps)[None, :] < first[:, None]

    hit = np.nonzero(first < n_steps)[0]
    hit_r, hit_c = rows[hit, first[hit]], cols[hit, first[hit]]
    on_map = inside[hit, first[hit]]   # running off the map edge isn't a wall
    free = (np.append(rows[visible], pos[0]), np.append(cols[visible], pos[1]))
    return free, (hit_r[on_map], hit_c[on_map])


def walls_between(occupied: np.ndarray, a, b) -> np.ndarray:
    """How many obstacles each straight line a[i] -> b[i] passes through.

    a and b are (n, 2) arrays of (row, col) cells. Counts entries into
    occupied cells along each line, sampled every half cell or finer.
    """
    a = np.asarray(a, dtype=float).reshape(-1, 2)
    b = np.asarray(b, dtype=float).reshape(-1, 2)
    if len(a) == 0:
        return np.zeros(0, dtype=int)
    longest = float(np.hypot(*(b - a).T).max())
    t = np.linspace(0.0, 1.0, max(2, math.ceil(longest * 2) + 1))
    h, w = occupied.shape
    rows = np.clip(np.floor(a[:, :1] + 0.5 + t * (b[:, :1] - a[:, :1])).astype(np.intp), 0, h - 1)
    cols = np.clip(np.floor(a[:, 1:] + 0.5 + t * (b[:, 1:] - a[:, 1:])).astype(np.intp), 0, w - 1)
    occ = occupied[rows, cols]
    return occ[:, 0].astype(int) + np.count_nonzero(occ[:, 1:] & ~occ[:, :-1], axis=1)


def traversable(blocked: np.ndarray, radius_cells: float, free: np.ndarray = None) -> np.ndarray:
    """Cells a robot of the given clearance can stand in.

    A cell qualifies if its center is more than radius_cells from every
    blocked cell's center, and (if `free` is given) it is known free.
    """
    clearance = ndimage.distance_transform_edt(~blocked)
    ok = clearance > radius_cells
    if free is not None:
        ok &= free
    return ok


# (row step, col step, length) for 8-connected moves; the other four are the reverses.
_MOVES = ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)))


def shortest_paths(trav: np.ndarray, start):
    """Dijkstra from `start` over traversable cells, 8-connected without cutting corners.

    Returns (dist, pred): dist is an (H, W) array of path lengths in cells
    (inf where unreachable); pred holds each flat cell's predecessor (< 0 at
    the start and for unreachable cells).
    """
    h, w = trav.shape
    index = np.arange(h * w).reshape(h, w)
    src, dst, length = [], [], []
    for dr, dc, step in _MOVES:
        ra, rb = slice(0, h - dr), slice(dr, h)
        ca, cb = (slice(0, w - dc), slice(dc, w)) if dc >= 0 else (slice(-dc, w), slice(0, w + dc))
        ok = trav[ra, ca] & trav[rb, cb]
        if dr and dc:   # a diagonal step needs both side cells clear too
            ok &= trav[rb, ca] & trav[ra, cb]
        src.append(index[ra, ca][ok])
        dst.append(index[rb, cb][ok])
        length.append(np.full(np.count_nonzero(ok), step))
    graph = csr_matrix(
        (np.concatenate(length), (np.concatenate(src), np.concatenate(dst))), shape=(h * w, h * w))
    dist, pred = dijkstra(graph, directed=False, indices=start[0] * w + start[1],
                          return_predecessors=True)
    return dist.reshape(h, w), pred


def path_to(pred: np.ndarray, start, goal, width: int):
    """Flat cell indices from just after `start` to `goal`, or None if unreachable."""
    start_i, node = start[0] * width + start[1], goal[0] * width + goal[1]
    path = []
    while node != start_i:
        if node < 0:
            return None
        path.append(int(node))
        node = pred[node]
    path.reverse()
    return path
