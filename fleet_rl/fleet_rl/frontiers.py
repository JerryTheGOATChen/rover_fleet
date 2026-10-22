"""Frontier detection: known-free cells that touch unknown space.

Same idea as rover_autonomy's frontier.hpp (the single-robot FSM), on the
coarse fleet grid.
"""

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from .config import FREE, UNKNOWN


@dataclass
class Frontier:
    cells: np.ndarray     # flat indices of the frontier's cells
    centroid: tuple       # (row, col), may fall between cells

    @property
    def size(self) -> int:
        return len(self.cells)


def find_frontiers(belief: np.ndarray, min_cells: int = 3):
    """Clusters (8-connected) of free cells with an unknown 4-neighbour."""
    unknown = belief == UNKNOWN
    touches_unknown = np.zeros_like(unknown)
    touches_unknown[1:, :] |= unknown[:-1, :]
    touches_unknown[:-1, :] |= unknown[1:, :]
    touches_unknown[:, 1:] |= unknown[:, :-1]
    touches_unknown[:, :-1] |= unknown[:, 1:]
    frontier = (belief == FREE) & touches_unknown

    labels, n = ndimage.label(frontier, structure=np.ones((3, 3), dtype=bool))
    if n == 0:
        return []
    flat = labels.ravel()
    cells = np.flatnonzero(flat)
    cells = cells[np.argsort(flat[cells], kind='stable')]
    groups = np.split(cells, np.cumsum(np.bincount(flat[cells])[1:])[:-1])

    width = belief.shape[1]
    out = []
    for group in groups:
        if len(group) >= min_cells:
            rows, cols = np.divmod(group, width)
            out.append(Frontier(cells=group, centroid=(rows.mean(), cols.mean())))
    return out
