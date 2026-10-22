"""Houses for the grid simulator.

generate_house(seed)  a random floor plan: rooms by recursive splitting, a door
                      in every split wall, a few extra doors for loops, furniture
load_sdf_house(path)  the Gazebo house, rasterized from the world's static boxes
"""

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

from .config import MAX_ROBOTS, HouseConfig, SimConfig
from .grid import traversable


@dataclass
class House:
    occupied: np.ndarray          # (H, W) bool ground truth
    cell: float                   # m per cell
    origin: tuple                 # world (x, y) of the grid's corner (row 0, col 0)
    base: tuple                   # (row, col) of the base station
    spawns: list = field(default_factory=list)   # (row, col) robot start cells
    name: str = ''

    @property
    def shape(self):
        return self.occupied.shape

    def to_world(self, cell):
        return (self.origin[0] + (cell[1] + 0.5) * self.cell,
                self.origin[1] + (cell[0] + 0.5) * self.cell)

    def to_cell(self, x, y):
        return (int(math.floor((y - self.origin[1]) / self.cell)),
                int(math.floor((x - self.origin[0]) / self.cell)))


# ---------------------------------------------------------------- generator

def generate_house(seed: int, hcfg: HouseConfig = HouseConfig(), sim: SimConfig = SimConfig()):
    """A random house. The same seed always gives the same house."""
    rng = np.random.default_rng(seed)
    for _ in range(100):
        house = _try_generate(rng, hcfg, sim)
        if house is not None:
            house.name = f'house-{seed}'
            return house
    raise RuntimeError(f'could not generate a valid house for seed {seed}')


def _try_generate(rng, hcfg: HouseConfig, sim: SimConfig):
    cell = sim.cell
    h = int(rng.uniform(*hcfg.height_m) / cell) + 2      # +2: outer walls
    w = int(rng.uniform(*hcfg.width_m) / cell) + 2
    occ = np.zeros((h, w), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    door_mask = np.zeros_like(occ)

    min_room = round(hcfg.min_room_m / cell)
    max_room = round(hcfg.max_room_m / cell)
    door = round(hcfg.door_m / cell)

    # Rooms are interiors [r0, r1) x [c0, c1), walled at r0-1, r1, c0-1, c1.
    rooms, stack = [], [(1, 1, h - 1, w - 1)]
    while stack:
        r0, c0, r1, c1 = stack.pop()
        rh, rw = r1 - r0, c1 - c0
        can_row, can_col = rh >= 2 * min_room + 1, rw >= 2 * min_room + 1
        must = max(rh, rw) > max_room
        if not (can_row or can_col) or (not must and rng.random() < 0.5):
            rooms.append((r0, c0, r1, c1))
            continue
        # Split across the longer side, so rooms stay roughly square.
        split_rows = can_row and (not can_col or rh > rw or (rh == rw and rng.random() < 0.5))
        s = _pick_wall(occ, rng, (r0, c0, r1, c1), split_rows, min_room)
        if s is None:
            rooms.append((r0, c0, r1, c1))
            continue
        if split_rows:
            occ[s, c0:c1] = True
            _carve_door(occ, door_mask, rng, True, s, c0, c1, door)
            stack += [(r0, c0, s, c1), (s + 1, c0, r1, c1)]
        else:
            occ[r0:r1, s] = True
            _carve_door(occ, door_mask, rng, False, s, r0, r1, door)
            stack += [(r0, c0, r1, s), (r0, s + 1, r1, c1)]

    _add_loop_doors(occ, door_mask, rng, rooms, door, hcfg.loop_door_prob)

    radius = sim.radius_cells
    if not _rooms_connected(occ, rooms, radius):
        return None
    _add_furniture(occ, door_mask, rng, rooms, radius, hcfg, cell)

    trav = traversable(occ, radius)
    labels, _ = ndimage.label(trav)
    r0, c0, r1, c1 = rooms[rng.integers(len(rooms))]
    center = np.array([(r0 + r1) / 2, (c0 + c1) / 2])
    cells = np.argwhere(trav[r0:r1, c0:c1]) + (r0, c0)
    if len(cells) == 0:
        return None
    base = tuple(int(v) for v in cells[np.argmin(np.hypot(*(cells - center).T))])
    spawns = _spawn_cells(trav & (labels == labels[base]), base, hcfg.spawn_radius_m / cell, rng)
    return House(occupied=occ, cell=cell, origin=(0.0, 0.0), base=base, spawns=spawns)


def _pick_wall(occ, rng, room, split_rows, min_room, margin=2):
    """A row (or column) to put a dividing wall on, or None.

    The new wall must not end right next to a door in the wall it meets,
    or it would cut that door in half.
    """
    r0, c0, r1, c1 = room
    lo, hi = (r0, r1) if split_rows else (c0, c1)
    options = list(range(lo + min_room, hi - min_room))
    rng.shuffle(options)
    for s in options:
        if split_rows:
            ends = occ[s - margin:s + margin + 1, c0 - 1], occ[s - margin:s + margin + 1, c1]
        else:
            ends = occ[r0 - 1, s - margin:s + margin + 1], occ[r1, s - margin:s + margin + 1]
        if ends[0].all() and ends[1].all():
            return s
    return None


def _carve_door(occ, door_mask, rng, horizontal, line, lo, hi, door):
    """Open `door` cells in a wall along row/column `line` spanning [lo, hi)."""
    start = rng.integers(lo + 1, hi - door)   # leave a wall cell at each end
    if horizontal:
        occ[line, start:start + door] = False
        door_mask[line, start:start + door] = True
    else:
        occ[start:start + door, line] = False
        door_mask[start:start + door, line] = True


def _add_loop_doors(occ, door_mask, rng, rooms, door, prob):
    """Extra doors between neighbouring rooms that don't already share one."""
    for i, (ar0, ac0, ar1, ac1) in enumerate(rooms):
        for br0, bc0, br1, bc1 in rooms[i + 1:]:
            if ar1 + 1 == br0 or br1 + 1 == ar0:          # stacked: shared wall row
                line = ar1 if ar1 + 1 == br0 else br1
                lo, hi, horizontal = max(ac0, bc0), min(ac1, bc1), True
                wall = occ[line, lo:hi]
            elif ac1 + 1 == bc0 or bc1 + 1 == ac0:        # side by side: shared wall column
                line = ac1 if ac1 + 1 == bc0 else bc1
                lo, hi, horizontal = max(ar0, br0), min(ar1, br1), False
                wall = occ[lo:hi, line]
            else:
                continue
            if hi - lo >= door + 2 and wall.all() and rng.random() < prob:
                _carve_door(occ, door_mask, rng, horizontal, line, lo, hi, door)


def _rooms_connected(occ, rooms, radius):
    """True if the robot can reach every room, all from one connected area."""
    labels, _ = ndimage.label(traversable(occ, radius))
    seen = set()
    for r0, c0, r1, c1 in rooms:
        room_labels = np.unique(labels[r0:r1, c0:c1])
        room_labels = room_labels[room_labels > 0]
        if len(room_labels) == 0:
            return False
        seen.update(room_labels.tolist())
    return len(seen) == 1


def _add_furniture(occ, door_mask, rng, rooms, radius, hcfg, cell):
    """0..N boxes per room, kept clear of doors and never cutting a room off."""
    door_dist = ndimage.distance_transform_edt(~door_mask)
    keep_clear = hcfg.door_clearance_m / cell
    for r0, c0, r1, c1 in rooms:
        for _ in range(rng.integers(0, hcfg.max_furniture_per_room + 1)):
            for _attempt in range(10):
                fh, fw = int(rng.integers(3, 9)), int(rng.integers(3, 11))
                if rng.random() < 0.5:
                    fh, fw = fw, fh
                if fh > (r1 - r0) - 6 or fw > (c1 - c0) - 6:
                    continue
                if rng.random() < 0.6:   # against a wall
                    side = rng.integers(4)
                    fr = r0 if side == 0 else r1 - fh if side == 1 else rng.integers(r0, r1 - fh + 1)
                    fc = (c0 if side == 2 else c1 - fw if side == 3
                          else rng.integers(c0, c1 - fw + 1))
                else:                    # free-standing, with room to walk around it
                    fr = rng.integers(r0 + 3, r1 - fh - 2)
                    fc = rng.integers(c0 + 3, c1 - fw - 2)
                block = (slice(fr, fr + fh), slice(fc, fc + fw))
                if door_dist[block].min() < keep_clear:
                    continue
                trial = occ.copy()
                trial[block] = True
                if _rooms_connected(trial, rooms, radius):
                    occ[block] = True
                    break


def _spawn_cells(region, base, radius_cells, rng):
    """MAX_ROBOTS start cells in `region` near the base (repeats allowed if few)."""
    cells = np.argwhere(region)
    near = cells[np.hypot(*(cells - np.array(base)).T) <= radius_cells]
    if len(near) == 0:
        near = np.array([base])
    picks = rng.choice(len(near), size=MAX_ROBOTS, replace=len(near) < MAX_ROBOTS)
    return [tuple(int(v) for v in near[i]) for i in picks]


# ---------------------------------------------------------------- Gazebo house

def gazebo_world_path():
    """rover_world.sdf from the source tree, or from the installed rover_bringup package."""
    here = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'rover_bringup',
                        'worlds', 'rover_world.sdf')
    if os.path.exists(here):
        return os.path.abspath(here)
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('rover_bringup'), 'worlds', 'rover_world.sdf')


# Where fleet.launch.py spawns robot1..6 in rover_world.sdf (checked to be
# >= 0.9 m from furniture and >= 1.3 m apart), and where the base station is.
GAZEBO_SPAWNS = [(0.0, 0.0), (-1.3, 0.0), (1.3, 0.0), (-1.3, 1.3), (0.0, 1.3), (-2.6, 0.0)]
GAZEBO_BASE = (0.0, 0.0)

def _pose(element):
    """(x, y, z, yaw) from an SDF <pose>, or zeros if absent."""
    text = element.findtext('pose') if element is not None else None
    v = [float(x) for x in text.split()] if text else [0.0] * 6
    return v[0], v[1], v[2], v[5]


def load_sdf_house(path, cell=0.2, base_xy=(0.0, 0.0), spawn_xys=(), lidar_height=0.40,
                   margin_m=0.6):
    """Rasterize a Gazebo world's static boxes and cylinders into a House.

    Only obstacles that cross the lidar's height are kept: lower ones are
    invisible to the lidar (and so to SLAM and Nav2's costmaps) anyway.
    A cell is occupied if any obstacle overlaps any part of it, so 0.2 m
    walls can't slip between cell centers.
    """
    shapes = []   # (x, y, yaw, half_x, half_y) in world coordinates
    for model in ET.parse(path).getroot().iter('model'):
        if (model.findtext('static') or '').strip() != 'true':
            continue
        mx, my, mz, myaw = _pose(model)
        for link in model.findall('link'):
            lx, ly, lz, lyaw = _pose(link)
            for collision in link.findall('collision'):
                cx, cy, cz, cyaw = _pose(collision)
                geometry = collision.find('geometry')
                box, cyl = geometry.find('box'), geometry.find('cylinder')
                if box is not None:
                    sx, sy, sz = (float(v) for v in box.findtext('size').split())
                elif cyl is not None:
                    radius = float(cyl.findtext('radius'))
                    sx = sy = 2 * radius
                    sz = float(cyl.findtext('length'))
                else:
                    continue   # ground plane etc.
                # Compose model -> link -> collision poses (planar).
                x, y, yaw = mx, my, myaw
                for ox, oy, oyaw in ((lx, ly, lyaw), (cx, cy, cyaw)):
                    x += ox * math.cos(yaw) - oy * math.sin(yaw)
                    y += ox * math.sin(yaw) + oy * math.cos(yaw)
                    yaw += oyaw
                z = mz + lz + cz
                if z - sz / 2 <= lidar_height <= z + sz / 2:
                    shapes.append((x, y, yaw, sx / 2, sy / 2))
    if not shapes:
        raise ValueError(f'no static obstacles found in {path}')

    extent = []
    for x, y, yaw, hx, hy in shapes:   # axis-aligned bounds of each (possibly rotated) shape
        ex = abs(math.cos(yaw)) * hx + abs(math.sin(yaw)) * hy
        ey = abs(math.sin(yaw)) * hx + abs(math.cos(yaw)) * hy
        extent.append((x - ex, y - ey, x + ex, y + ey))
    extent = np.array(extent)
    x0 = math.floor((extent[:, 0].min() - margin_m) / cell) * cell
    y0 = math.floor((extent[:, 1].min() - margin_m) / cell) * cell
    w = math.ceil((extent[:, 2].max() + margin_m - x0) / cell)
    h = math.ceil((extent[:, 3].max() + margin_m - y0) / cell)

    occ = np.zeros((h, w), dtype=bool)
    xs = x0 + (np.arange(w) + 0.5) * cell
    ys = y0 + (np.arange(h) + 0.5) * cell
    gx, gy = np.meshgrid(xs, ys)
    for x, y, yaw, hx, hy in shapes:
        # Cell centers in the shape's frame; growing the shape by half a cell
        # marks every cell the shape overlaps (exact for axis-aligned boxes).
        dx, dy = gx - x, gy - y
        lx = dx * math.cos(yaw) + dy * math.sin(yaw)
        ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
        occ |= (np.abs(lx) < hx + cell / 2) & (np.abs(ly) < hy + cell / 2)

    house = House(occupied=occ, cell=cell, origin=(x0, y0), base=(0, 0), name='gazebo-house')
    house.base = house.to_cell(*base_xy)
    house.spawns = [house.to_cell(x, y) for x, y in spawn_xys] or [house.base]
    if occ[house.base]:
        raise ValueError(f'base {base_xy} is inside an obstacle')
    return house
