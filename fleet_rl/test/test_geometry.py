"""Grid geometry, frontiers and the radio model."""

import math

import numpy as np

from fleet_rl.comms import connected_groups, link_matrix, merge_maps, signal_margin
from fleet_rl.config import FREE, OCCUPIED, UNKNOWN
from fleet_rl.frontiers import find_frontiers
from fleet_rl.grid import path_to, raycast, shortest_paths, traversable, walls_between


def room(h=30, w=40):
    occ = np.zeros((h, w), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    return occ


def test_lidar_stops_at_walls():
    occ = room()
    occ[:, 20] = True                      # a full wall at column 20
    free, hit = raycast(occ, (15, 10), n_rays=360, range_cells=50)
    assert free[1].max() < 20              # nothing seen beyond the wall
    assert (hit[1] == 20).any()            # the wall itself is seen
    assert occ[hit].all() and not occ[free].any()


def test_lidar_range_limit():
    free, _ = raycast(room(100, 100), (50, 50), n_rays=360, range_cells=10)
    assert np.hypot(free[0] - 50, free[1] - 50).max() <= 11


def test_walls_between_counts_each_wall():
    occ = room()
    occ[:, 15] = True
    occ[:, 25] = True
    a = [(10, 5), (10, 5), (10, 5)]
    b = [(10, 12), (10, 20), (10, 30)]
    assert walls_between(occ, a, b).tolist() == [0, 1, 2]


def test_traversable_keeps_clearance_and_door_passable():
    occ = room()
    occ[:, 20] = True
    occ[12:20, 20] = False                 # an 8-cell (1.6 m) door
    radius = 0.45 / 0.2 + 0.5
    trav = traversable(occ, radius)
    assert not trav[1, :].any()            # hugging the outer wall is too tight
    assert trav[15:17, 20].all()           # the middle of the door is passable
    dist, _ = shortest_paths(trav, (15, 10))
    assert np.isfinite(dist[15, 30])       # so the far room is reachable


def test_shortest_paths_lengths_and_path():
    trav = np.ones((20, 20), dtype=bool)
    dist, pred = shortest_paths(trav, (0, 0))
    assert dist[0, 10] == 10
    assert math.isclose(dist[10, 10], 10 * math.sqrt(2))
    path = path_to(pred, (0, 0), (5, 7), 20)
    assert path[-1] == 5 * 20 + 7 and len(path) == 7
    trav[:, 10] = False                    # cut the grid in two
    dist, pred = shortest_paths(trav, (0, 0))
    assert np.isinf(dist[0, 15]) and path_to(pred, (0, 0), (0, 15), 20) is None


def test_no_corner_cutting():
    trav = np.ones((3, 3), dtype=bool)
    trav[0, 1] = trav[1, 0] = False        # the diagonal (0,0)->(1,1) squeezes between two walls
    dist, _ = shortest_paths(trav, (0, 0))
    assert np.isinf(dist[1, 1])


def test_frontiers_found_and_small_ones_dropped():
    belief = np.full((20, 20), UNKNOWN, dtype=np.int8)
    belief[:, :10] = FREE                  # left half known: one long frontier at column 9
    belief[5, 15] = FREE                   # an isolated speck
    frontiers = find_frontiers(belief, min_cells=3)
    assert len(frontiers) == 1
    assert frontiers[0].size == 20
    assert math.isclose(frontiers[0].centroid[1], 9)


def test_link_rule_distance_and_walls():
    occ = room(30, 80)
    cell = 0.2
    # 4 m apart in the open: linked with a 5 m range, not with a 3 m range.
    pts = [(15, 10), (15, 30)]
    assert link_matrix(pts, occ, 5.0, 3.0, cell)[0, 1]
    assert not link_matrix(pts, occ, 3.0, 3.0, cell)[0, 1]
    # A wall in between costs 3 m: 4 + 3 = 7 > 5.
    occ[:, 20] = True
    assert not link_matrix(pts, occ, 5.0, 3.0, cell)[0, 1]
    assert link_matrix(pts, occ, 7.5, 3.0, cell)[0, 1]
    assert link_matrix(pts, occ, math.inf, 3.0, cell).all()


def test_groups_chain_through_relays():
    adj = np.eye(4, dtype=bool)
    adj[0, 1] = adj[1, 0] = True           # 0 - 1 - 2 chained, 3 alone
    adj[1, 2] = adj[2, 1] = True
    assert connected_groups(adj) == [[0, 1, 2], [3]]


def test_merge_maps():
    a = np.array([UNKNOWN, FREE, FREE, UNKNOWN], dtype=np.int8)
    b = np.array([UNKNOWN, UNKNOWN, OCCUPIED, FREE], dtype=np.int8)
    assert merge_maps([a, b]).tolist() == [UNKNOWN, FREE, OCCUPIED, FREE]


def test_signal_margin():
    blocked = np.zeros((10, 100), dtype=bool)
    m = signal_margin(blocked, [(5, 5), (5, 30), (5, 80)], [(5, 5)], 5.0, 3.0, 0.2)
    assert math.isclose(m[0], 1.0)
    assert math.isclose(m[1], 0.0, abs_tol=1e-9)   # exactly 5 m away
    assert m[2] == -1.0
