"""Converting between ROS maps and the fleet's world grid.

The fleet shares maps on one fixed grid in the `world` frame (0.2 m cells,
covering the Gazebo house), the same grid the policy was trained on. Each
robot's SLAM map lives in its own frame and resolution and is resampled onto
that grid before it is used or sent anywhere.
"""

import math
from dataclasses import dataclass

import numpy as np
from nav_msgs.msg import OccupancyGrid

from fleet_rl.config import FREE, OCCUPIED, UNKNOWN


@dataclass(frozen=True)
class GridGeometry:
    origin_x: float     # world x of the grid's corner (row 0, col 0)
    origin_y: float
    cell: float         # m per cell
    width: int          # columns (x)
    height: int         # rows (y)

    def to_cell(self, x, y):
        """(row, col) of a world point, clipped onto the grid."""
        row = int(math.floor((y - self.origin_y) / self.cell))
        col = int(math.floor((x - self.origin_x) / self.cell))
        return min(max(row, 0), self.height - 1), min(max(col, 0), self.width - 1)

    def to_world(self, cell):
        return (self.origin_x + (cell[1] + 0.5) * self.cell,
                self.origin_y + (cell[0] + 0.5) * self.cell)


def grid_to_msg(grid, geom: GridGeometry, stamp, frame_id='world') -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.info.resolution = float(geom.cell)
    msg.info.width, msg.info.height = geom.width, geom.height
    msg.info.origin.position.x = float(geom.origin_x)
    msg.info.origin.position.y = float(geom.origin_y)
    msg.info.origin.orientation.w = 1.0
    data = np.full(grid.shape, -1, dtype=np.int8)
    data[grid == FREE] = 0
    data[grid == OCCUPIED] = 100
    msg.data = data.ravel().tolist()
    return msg


def msg_to_grid(msg: OccupancyGrid, geom: GridGeometry):
    """Back to an int8 UNKNOWN/FREE/OCCUPIED grid (the message must use `geom`)."""
    if (msg.info.width, msg.info.height) != (geom.width, geom.height):
        raise ValueError(f'map is {msg.info.width}x{msg.info.height}, '
                         f'expected {geom.width}x{geom.height}')
    raw = np.asarray(msg.data, dtype=np.int16).reshape(geom.height, geom.width)
    grid = np.full(raw.shape, UNKNOWN, dtype=np.int8)
    grid[(raw >= 0) & (raw < 50)] = FREE
    grid[raw >= 50] = OCCUPIED
    return grid


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def slam_to_world(msg: OccupancyGrid, map_pose, geom: GridGeometry, occupied_min=2, free_min=4):
    """Resample a SLAM map onto the world grid.

    map_pose = (x, y, yaw) of the SLAM map's frame in the world. Each world
    cell counts the SLAM cells that land in it: occupied if at least
    `occupied_min` are walls (one stray cell is noise), else free if at least
    `free_min` are free, else unknown. With 0.05 m SLAM cells a 0.2 m world
    cell holds 16 of them.
    """
    w, h, res = msg.info.width, msg.info.height, msg.info.resolution
    raw = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
    rows, cols = np.nonzero(raw >= 0)
    values = raw[rows, cols]

    # Cell centers: map-origin frame -> map frame -> world.
    ox, oy, oyaw = msg.info.origin.position.x, msg.info.origin.position.y, \
        _yaw(msg.info.origin.orientation)
    mx, my = (cols + 0.5) * res, (rows + 0.5) * res
    x = ox + math.cos(oyaw) * mx - math.sin(oyaw) * my
    y = oy + math.sin(oyaw) * mx + math.cos(oyaw) * my
    px, py, pyaw = map_pose
    wx = px + math.cos(pyaw) * x - math.sin(pyaw) * y
    wy = py + math.sin(pyaw) * x + math.cos(pyaw) * y

    r = np.floor((wy - geom.origin_y) / geom.cell).astype(np.intp)
    c = np.floor((wx - geom.origin_x) / geom.cell).astype(np.intp)
    inside = (r >= 0) & (r < geom.height) & (c >= 0) & (c < geom.width)
    index, values = r[inside] * geom.width + c[inside], values[inside]
    n = geom.width * geom.height
    occupied = np.bincount(index[values >= 65], minlength=n)
    free = np.bincount(index[values < 25], minlength=n)

    grid = np.full(n, UNKNOWN, dtype=np.int8)
    grid[free >= free_min] = FREE
    grid[occupied >= occupied_min] = OCCUPIED
    return grid.reshape(geom.height, geom.width)
