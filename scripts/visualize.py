#!/usr/bin/env python3
"""
Plot a grid world with an overlaid path.
Reads the grid from a text file and the path from stdin or a file.

Usage:
    ./planner --grid scenarios/warehouse.txt --start 0,0 --goal 19,19 --algo astar | \
        python3 scripts/visualize.py --grid scenarios/warehouse.txt
"""

import argparse
import sys
import re
import numpy as np
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
    return np.array(rows, dtype=int)


def parse_path(text):
    """Extract (x,y) pairs from planner output."""
    coords = re.findall(r'\((\d+),(\d+)\)', text)
    return [(int(x), int(y)) for x, y in coords]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--grid', required=True, help='grid file')
    parser.add_argument('--path-file', help='file with planner output (default: stdin)')
    parser.add_argument('-o', '--output', help='save to file instead of showing')
    args = parser.parse_args()

    grid = load_grid(args.grid)

    if args.path_file:
        with open(args.path_file) as f:
            path_text = f.read()
    else:
        path_text = sys.stdin.read()

    path = parse_path(path_text)

    # color map: white=free, dark gray=blocked, orange=dynamic
    cmap = mcolors.ListedColormap(['#f0f0f0', '#2d2d2d', '#e8873a'])
    bounds = [-0.5, 0.5, 1.5, 2.5]
    norm = mcolors.BoundaryNorm(bounds, cmap.N)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(grid, cmap=cmap, norm=norm, origin='upper')

    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, '-', color='#2196F3', linewidth=2, alpha=0.8)
        ax.plot(xs[0], ys[0], 'o', color='#4CAF50', markersize=10, label='start')
        ax.plot(xs[-1], ys[-1], 's', color='#F44336', markersize=10, label='goal')
        ax.legend(loc='upper right')

    ax.set_xlim(-0.5, grid.shape[1] - 0.5)
    ax.set_ylim(grid.shape[0] - 0.5, -0.5)
    ax.set_aspect('equal')
    ax.set_title(f'path: {len(path)} waypoints')

    if args.output:
        plt.savefig(args.output, dpi=150, bbox_inches='tight')
        print(f'saved to {args.output}')
    else:
        plt.show()


if __name__ == '__main__':
    main()
