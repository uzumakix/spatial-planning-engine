#!/usr/bin/env python3
"""Generate result images for the README. Runs a simple Python A* on the
scenario grids and plots the paths using the same style as visualize.py."""

import heapq
import math
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors


def load_grid(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.rstrip('\n')
            if not line:
                continue
            row = []
            for ch in line:
                if ch == '#':
                    row.append(1)
                elif ch == 'D':
                    row.append(2)
                else:
                    row.append(0)
            rows.append(row)
    # pad rows to uniform width
    max_w = max(len(r) for r in rows)
    for r in rows:
        while len(r) < max_w:
            r.append(0)
    return np.array(rows, dtype=int)


def astar(grid, start, goal, eight_conn=False):
    h, w = grid.shape
    def passable(x, y):
        return 0 <= x < w and 0 <= y < h and grid[y, x] == 0

    if eight_conn:
        dirs = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
    else:
        dirs = [(1,0),(-1,0),(0,1),(0,-1)]

    sx, sy = start
    gx, gy = goal
    dist = {(sx, sy): 0.0}
    parent = {}
    pq = [(math.hypot(gx - sx, gy - sy), 0.0, sx, sy)]
    closed = set()

    while pq:
        f, g, x, y = heapq.heappop(pq)
        if (x, y) in closed:
            continue
        closed.add((x, y))
        if (x, y) == (gx, gy):
            path = []
            cur = (gx, gy)
            while cur is not None:
                path.append(cur)
                cur = parent.get(cur)
            return list(reversed(path))

        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if not passable(nx, ny):
                continue
            if eight_conn and abs(dx) + abs(dy) == 2:
                if not passable(x + dx, y) or not passable(x, y + dy):
                    continue
            cost = math.sqrt(dx*dx + dy*dy)
            ng = g + cost
            if ng < dist.get((nx, ny), float('inf')):
                dist[(nx, ny)] = ng
                parent[(nx, ny)] = (x, y)
                heur = math.hypot(gx - nx, gy - ny)
                heapq.heappush(pq, (ng + heur, ng, nx, ny))
    return None


def plot_grid_path(grid, path, title, output_path):
    cmap = mcolors.ListedColormap(['#f0f0f0', '#2d2d2d', '#e8873a'])
    bounds = [-0.5, 0.5, 1.5, 2.5]
    norm = mcolors.BoundaryNorm(bounds, cmap.N)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(grid, cmap=cmap, norm=norm, origin='upper')

    # grid lines
    for i in range(grid.shape[0] + 1):
        ax.axhline(i - 0.5, color='#cccccc', linewidth=0.3)
    for j in range(grid.shape[1] + 1):
        ax.axvline(j - 0.5, color='#cccccc', linewidth=0.3)

    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, '-', color='#2196F3', linewidth=2.5, alpha=0.85)
        ax.plot(xs[0], ys[0], 'o', color='#4CAF50', markersize=12,
                zorder=5, label='start')
        ax.plot(xs[-1], ys[-1], 's', color='#F44336', markersize=12,
                zorder=5, label='goal')
        ax.legend(loc='upper right', fontsize=11)

    ax.set_xlim(-0.5, grid.shape[1] - 0.5)
    ax.set_ylim(grid.shape[0] - 0.5, -0.5)
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=13, pad=10)
    ax.set_xticks([])
    ax.set_yticks([])

    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    print(f'saved {output_path}')


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    proj_dir = os.path.dirname(script_dir)
    results_dir = os.path.join(proj_dir, 'results')
    os.makedirs(results_dir, exist_ok=True)

    # maze: A* 4-connected
    maze = load_grid(os.path.join(proj_dir, 'scenarios', 'maze.txt'))
    maze_path = astar(maze, (0, 0), (14, 14), eight_conn=False)
    plot_grid_path(maze, maze_path,
                   f'A* on 15x15 maze ({len(maze_path)} waypoints)',
                   os.path.join(results_dir, 'maze_astar.png'))

    # warehouse: A* 8-connected
    wh = load_grid(os.path.join(proj_dir, 'scenarios', 'warehouse.txt'))
    wh_path = astar(wh, (0, 0), (19, 19), eight_conn=True)
    plot_grid_path(wh, wh_path,
                   f'A* (8-conn) on 20x20 warehouse ({len(wh_path)} waypoints)',
                   os.path.join(results_dir, 'warehouse_astar.png'))


if __name__ == '__main__':
    main()
