#!/usr/bin/env python3
"""Generate a ground-truth occupancy grid map for the closed-8 track.

Track geometry (from track.sdf):
- Outer walls: x ∈ [-10, 10], y ∈ [-12, 12], thickness 0.15m
- Upper island: x ∈ [-5, 5], y ∈ [4, 8], thickness 0.15m
- Lower island: x ∈ [-5, 5], y ∈ [-8, -4], thickness 0.15m
- Center square block: x ∈ [-1.5, 1.5], y ∈ [-1.5, 1.5], thickness 0.15m
- Static obstacles: 2 boxes + 2 cylinders
"""

import numpy as np
from PIL import Image
import os

# Map parameters
RESOLUTION = 0.05  # meters per pixel
ORIGIN_X = -14.0
ORIGIN_Y = -16.0
WIDTH_M = 28.0
HEIGHT_M = 32.0

WIDTH_PX = int(WIDTH_M / RESOLUTION)   # 560
HEIGHT_PX = int(HEIGHT_M / RESOLUTION)  # 640

FREE = 254
OCCUPIED = 0
UNKNOWN = 205

WALL_THICKNESS = 0.15


def world_to_pixel(wx, wy):
    col = int((wx - ORIGIN_X) / RESOLUTION)
    row = int((wy - ORIGIN_Y) / RESOLUTION)
    row = HEIGHT_PX - 1 - row
    return row, col


def draw_rect(grid, x_min, y_min, x_max, y_max):
    for wx in np.arange(x_min, x_max, RESOLUTION / 2):
        for wy in np.arange(y_min, y_max, RESOLUTION / 2):
            r, c = world_to_pixel(wx, wy)
            if 0 <= r < HEIGHT_PX and 0 <= c < WIDTH_PX:
                grid[r, c] = OCCUPIED


def draw_circle(grid, cx, cy, radius):
    for wx in np.arange(cx - radius, cx + radius, RESOLUTION / 2):
        for wy in np.arange(cy - radius, cy + radius, RESOLUTION / 2):
            if (wx - cx) ** 2 + (wy - cy) ** 2 <= radius ** 2:
                r, c = world_to_pixel(wx, wy)
                if 0 <= r < HEIGHT_PX and 0 <= c < WIDTH_PX:
                    grid[r, c] = OCCUPIED


def fill_region_unknown(grid, x_min, y_min, x_max, y_max):
    """Fill a rectangular region as unknown (inside islands/blocks)."""
    for r in range(HEIGHT_PX):
        for c in range(WIDTH_PX):
            wx = c * RESOLUTION + ORIGIN_X
            wy = (HEIGHT_PX - 1 - r) * RESOLUTION + ORIGIN_Y
            if x_min < wx < x_max and y_min < wy < y_max:
                grid[r, c] = UNKNOWN


def main():
    grid = np.full((HEIGHT_PX, WIDTH_PX), FREE, dtype=np.uint8)

    t = WALL_THICKNESS

    # Mark outside outer walls as unknown
    for r in range(HEIGHT_PX):
        for c in range(WIDTH_PX):
            wx = c * RESOLUTION + ORIGIN_X
            wy = (HEIGHT_PX - 1 - r) * RESOLUTION + ORIGIN_Y
            if wx < -10.0 or wx > 10.0 or wy < -12.0 or wy > 12.0:
                grid[r, c] = UNKNOWN

    # ===== OUTER WALLS (x ∈ [-10, 10], y ∈ [-12, 12]) =====
    draw_rect(grid, -10.0, 12.0 - t/2, 10.0, 12.0 + t/2)     # north
    draw_rect(grid, -10.0, -12.0 - t/2, 10.0, -12.0 + t/2)    # south
    draw_rect(grid, 10.0 - t/2, -12.0, 10.0 + t/2, 12.0)      # east
    draw_rect(grid, -10.0 - t/2, -12.0, -10.0 + t/2, 12.0)    # west

    # ===== UPPER ISLAND (x ∈ [-5, 5], y ∈ [4, 8]) =====
    draw_rect(grid, -5.0, 8.0 - t/2, 5.0, 8.0 + t/2)          # north
    draw_rect(grid, -5.0, 4.0 - t/2, 5.0, 4.0 + t/2)          # south
    draw_rect(grid, 5.0 - t/2, 4.0, 5.0 + t/2, 8.0)           # east
    draw_rect(grid, -5.0 - t/2, 4.0, -5.0 + t/2, 8.0)         # west
    fill_region_unknown(grid, -5.0, 4.0, 5.0, 8.0)

    # ===== LOWER ISLAND (x ∈ [-5, 5], y ∈ [-8, -4]) =====
    draw_rect(grid, -5.0, -4.0 - t/2, 5.0, -4.0 + t/2)        # north
    draw_rect(grid, -5.0, -8.0 - t/2, 5.0, -8.0 + t/2)        # south
    draw_rect(grid, 5.0 - t/2, -8.0, 5.0 + t/2, -4.0)         # east
    draw_rect(grid, -5.0 - t/2, -8.0, -5.0 + t/2, -4.0)       # west
    fill_region_unknown(grid, -5.0, -8.0, 5.0, -4.0)

    # ===== CENTER SQUARE BLOCK (x ∈ [-1.5, 1.5], y ∈ [-1.5, 1.5]) =====
    draw_rect(grid, -1.5, 1.5 - t/2, 1.5, 1.5 + t/2)          # north
    draw_rect(grid, -1.5, -1.5 - t/2, 1.5, -1.5 + t/2)        # south
    draw_rect(grid, 1.5 - t/2, -1.5, 1.5 + t/2, 1.5)          # east
    draw_rect(grid, -1.5 - t/2, -1.5, -1.5 + t/2, 1.5)        # west
    fill_region_unknown(grid, -1.5, -1.5, 1.5, 1.5)

    # ===== STATIC OBSTACLES =====
    # Box at (3, -10) - bottom corridor, size 0.8 x 0.4
    draw_rect(grid, 3.0 - 0.4, -10.0 - 0.2, 3.0 + 0.4, -10.0 + 0.2)
    # Box at (-3, -10) - bottom corridor, size 0.8 x 0.4
    draw_rect(grid, -3.0 - 0.4, -10.0 - 0.2, -3.0 + 0.4, -10.0 + 0.2)
    # Cylinder at (7.5, 6) - right corridor, radius 0.3
    draw_circle(grid, 7.5, 6.0, 0.3)
    # Cylinder at (-7.5, -6) - left corridor, radius 0.3
    draw_circle(grid, -7.5, -6.0, 0.3)

    # Save PGM
    script_dir = os.path.dirname(os.path.abspath(__file__))
    maps_dir = os.path.join(script_dir, '..', 'maps')
    os.makedirs(maps_dir, exist_ok=True)

    pgm_path = os.path.join(maps_dir, 'ground_truth_map.pgm')
    img = Image.fromarray(grid, mode='L')
    img.save(pgm_path)
    print(f'Saved {pgm_path} ({WIDTH_PX}x{HEIGHT_PX})')

    yaml_path = os.path.join(maps_dir, 'ground_truth_map.yaml')
    with open(yaml_path, 'w') as f:
        f.write(f'image: ground_truth_map.pgm\n')
        f.write(f'mode: trinary\n')
        f.write(f'resolution: {RESOLUTION}\n')
        f.write(f'origin: [{ORIGIN_X}, {ORIGIN_Y}, 0]\n')
        f.write(f'negate: 0\n')
        f.write(f'occupied_thresh: 0.65\n')
        f.write(f'free_thresh: 0.10\n')
    print(f'Saved {yaml_path}')

    n_free = np.sum(grid == FREE)
    n_occ = np.sum(grid == OCCUPIED)
    n_unk = np.sum(grid == UNKNOWN)
    total = WIDTH_PX * HEIGHT_PX
    print(f'Free: {n_free} ({100 * n_free / total:.1f}%)')
    print(f'Occupied: {n_occ} ({100 * n_occ / total:.1f}%)')
    print(f'Unknown: {n_unk} ({100 * n_unk / total:.1f}%)')


if __name__ == '__main__':
    main()
