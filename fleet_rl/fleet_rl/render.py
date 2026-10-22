"""Draw episodes: record snapshots while a FleetSim runs, then save a GIF or PNG.

Picture legend: light floor = what the base knows; tinted floor = what a
robot knows but hasn't reported yet; faint outlines = walls nobody has seen;
green lines = radio links; yellow diamonds = beacons; white square = base.
"""

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .config import FREE, OCCUPIED

BACKGROUND = (21, 26, 36)
HIDDEN_WALL = (44, 51, 66)
KNOWN_FLOOR = (233, 228, 214)
KNOWN_WALL = (84, 90, 104)
LINK = (76, 195, 138)
BEACON = (242, 201, 76)
TEXT = (230, 233, 240)
ROBOT_COLORS = [(255, 138, 61), (61, 196, 255), (232, 93, 197), (150, 220, 80),
                (255, 214, 64), (160, 130, 255)]
HEADER_PX = 22


class Recorder:
    """Attach to a sim (sim.on_tick = recorder) to keep a snapshot every `every` ticks."""

    def __init__(self, every: int = 2):
        self.every, self.frames, self._ticks = every, [], 0

    def __call__(self, sim):
        if self._ticks % self.every == 0 or sim.done:
            self.frames.append(snapshot(sim))
        self._ticks += 1


def snapshot(sim):
    alive = [r for r in sim.robots if r.alive]
    return {
        't': sim.t,
        'coverage': sim.base_coverage,
        'base_belief': sim.base_belief.copy(),
        'robot_free': [r.belief == FREE for r in sim.robots],
        'robots': [(r.pos, r.alive, r.target) for r in sim.robots],
        'nodes': [r.pos for r in alive] + [sim.house.base] + list(sim.beacons),
        'links': sim.links.copy(),
        'beacons': list(sim.beacons),
    }


def _font(size):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:   # Pillow < 10.1
        return ImageFont.load_default()


def draw_frame(house, frame, scale: int = 6, title: str = '') -> Image.Image:
    h, w = house.shape
    rgb = np.empty((h, w, 3), dtype=np.uint8)
    rgb[:] = BACKGROUND
    rgb[house.occupied] = HIDDEN_WALL
    base_free = frame['base_belief'] == FREE
    for color, known in zip(ROBOT_COLORS, frame['robot_free']):
        unreported = known & ~base_free
        rgb[unreported] = (0.45 * np.array(color) + 0.55 * np.array(BACKGROUND)).astype(np.uint8)
    rgb[base_free] = KNOWN_FLOOR
    rgb[frame['base_belief'] == OCCUPIED] = KNOWN_WALL

    # Row 0 is the bottom of the map (+y up), so flip for the image.
    img = Image.fromarray(rgb[::-1]).resize((w * scale, h * scale), Image.NEAREST)
    canvas = Image.new('RGB', (w * scale, h * scale + HEADER_PX), BACKGROUND)
    canvas.paste(img, (0, HEADER_PX))
    draw = ImageDraw.Draw(canvas)

    def px(cell):
        return (cell[1] + 0.5) * scale, (h - cell[0] - 0.5) * scale + HEADER_PX

    nodes, links = frame['nodes'], frame['links']
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if links[i, j]:
                draw.line([px(nodes[i]), px(nodes[j])], fill=LINK, width=2)
    for b in frame['beacons']:
        x, y = px(b)
        r = scale * 1.2
        draw.polygon([(x, y - r), (x + r, y), (x, y + r), (x - r, y)], fill=BEACON)
    bx, by = px(house.base)
    r = scale * 1.4
    draw.rectangle([bx - r, by - r, bx + r, by + r], fill=(255, 255, 255), outline=(0, 0, 0))
    for color, (pos, alive, target) in zip(ROBOT_COLORS, frame['robots']):
        x, y = px(pos)
        r = scale * 1.6
        if not alive:
            draw.line([x - r, y - r, x + r, y + r], fill=(150, 150, 150), width=3)
            draw.line([x - r, y + r, x + r, y - r], fill=(150, 150, 150), width=3)
            continue
        if target is not None:
            tx, ty = px(target)
            draw.ellipse([tx - scale, ty - scale, tx + scale, ty + scale], outline=color, width=2)
        draw.ellipse([x - r, y - r, x + r, y + r], fill=color, outline=(0, 0, 0))

    header = f"t = {frame['t']:5.0f} s    base knows {100 * frame['coverage']:3.0f}%"
    if title:
        header = f'{title}    {header}'
    draw.text((6, 4), header, fill=TEXT, font=_font(14))
    return canvas


def save_gif(house, frames, path, scale: int = 6, title: str = '', ms_per_frame: int = 60):
    images = [draw_frame(house, f, scale, title) for f in frames]
    images += [images[-1]] * 25   # hold the last frame
    images[0].save(path, save_all=True, append_images=images[1:], duration=ms_per_frame,
                   loop=0, optimize=True)


def save_side_by_side(house, frames_a, frames_b, titles, path, scale: int = 5,
                      ms_per_frame: int = 60):
    """Two episodes on the same house, played in sync (the shorter one holds its last frame)."""
    n = max(len(frames_a), len(frames_b))
    images = []
    for i in range(n):
        a = draw_frame(house, frames_a[min(i, len(frames_a) - 1)], scale, titles[0])
        b = draw_frame(house, frames_b[min(i, len(frames_b) - 1)], scale, titles[1])
        gap = 8
        both = Image.new('RGB', (a.width + gap + b.width, max(a.height, b.height)), BACKGROUND)
        both.paste(a, (0, 0))
        both.paste(b, (a.width + gap, 0))
        images.append(both)
    images += [images[-1]] * 25
    images[0].save(path, save_all=True, append_images=images[1:], duration=ms_per_frame,
                   loop=0, optimize=True)
