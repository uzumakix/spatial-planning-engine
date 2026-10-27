"""
Independent reference solve of the bundled scenarios.

The planner is written in C++. This is a second, deliberately simple A* in
Python, used to check that the optimal path length the C++ reports is really
the optimal one. Two implementations agreeing is worth more than one
implementation asserting.

Run:  python scripts/check_paths.py
"""

from __future__ import annotations

import heapq
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def load(path):
    rows = [line.rstrip("\n") for line in path.read_text().splitlines() if line.strip()]
    # the scenario files are not padded to a rectangle; pad short rows as free space
    width = max(len(r) for r in rows)
    return [list(r.ljust(width, ".")) for r in rows]


def astar(grid, start, goal, eight):
    h_, w_ = len(grid), len(grid[0])

    def passable(x, y):
        return 0 <= x < w_ and 0 <= y < h_ and grid[y][x] != "#"

    if eight:
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                 (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]
    else:
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]

    def heur(a, b):
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return math.hypot(dx, dy) if eight else dx + dy

    openq = [(heur(start, goal), 0.0, start, None)]
    came, gscore = {}, {start: 0.0}
    while openq:
        _, g, cur, parent = heapq.heappop(openq)
        if cur in came:
            continue
        came[cur] = parent
        if cur == goal:
            path, node = [], cur
            while node is not None:
                path.append(node)
                node = came[node]
            return g, path[::-1]
        for dx, dy, cost in moves:
            nx, ny = cur[0] + dx, cur[1] + dy
            if not passable(nx, ny):
                continue
            # a diagonal step may not cut the corner of a blocked cell
            if dx and dy and not (passable(cur[0] + dx, cur[1]) and passable(cur[0], cur[1] + dy)):
                continue
            ng = g + cost
            if ng < gscore.get((nx, ny), float("inf")):
                gscore[(nx, ny)] = ng
                heapq.heappush(openq, (ng + heur((nx, ny), goal), ng, (nx, ny), cur))
    return None, []


for name, start, goal in [("maze", (0, 0), (14, 14)), ("warehouse", (0, 0), (19, 19))]:
    grid = load(ROOT / "scenarios" / f"{name}.txt")
    print(f"\n{name}: {len(grid[0])} x {len(grid)} cells")
    for eight in (False, True):
        cost, path = astar(grid, start, goal, eight)
        tag = "8-connected" if eight else "4-connected"
        if cost is None:
            print(f"  {tag}: no path")
        else:
            print(f"  {tag}: cost {cost:.3f}, {len(path)} waypoints")
