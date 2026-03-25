#include "grid.h"
#include "astar.h"
#include "rrt.h"
#include <iostream>
#include <chrono>
#include <cstdio>

int main() {
    const int SZ = 500;
    Grid grid(SZ, SZ);

    // scatter some rectangular obstacles to make it interesting
    // deterministic pattern: blocks every 20 cells with 10-wide walls
    for (int bx = 30; bx < SZ; bx += 40) {
        for (int y = 10; y < SZ - 10; y++) {
            for (int x = bx; x < bx + 10 && x < SZ; x++)
                grid.set(x, y, CellState::BLOCKED);
        }
        // leave a gap in each wall
        for (int x = bx; x < bx + 10 && x < SZ; x++) {
            grid.set(x, bx % 80 == 30 ? SZ / 3 : 2 * SZ / 3, CellState::FREE);
            grid.set(x, (bx % 80 == 30 ? SZ / 3 : 2 * SZ / 3) + 1, CellState::FREE);
        }
    }

    Coord start{5, SZ / 2};
    Coord goal{SZ - 5, SZ / 2};

    std::printf("benchmark: %dx%d grid with vertical wall obstacles\n\n", SZ, SZ);
    std::printf("%-10s %10s %10s %12s %10s\n",
                "algo", "cost", "waypoints", "nodes/iters", "time(ms)");
    std::printf("%-10s %10s %10s %12s %10s\n",
                "----", "----", "---------", "-----------", "-------");

    // A* (4-connected)
    {
        AStarPlanner planner(grid);
        auto t0 = std::chrono::high_resolution_clock::now();
        auto r = planner.plan(start, goal);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (r.found) {
            std::printf("%-10s %10.1f %10zu %12d %10.2f\n",
                        "A*-4", r.cost, r.path.size(), r.nodes_expanded, ms);
        } else {
            std::printf("%-10s %10s\n", "A*-4", "no path");
        }
    }

    // A* (8-connected)
    {
        AStarPlanner planner(grid, true);
        auto t0 = std::chrono::high_resolution_clock::now();
        auto r = planner.plan(start, goal);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (r.found) {
            std::printf("%-10s %10.1f %10zu %12d %10.2f\n",
                        "A*-8", r.cost, r.path.size(), r.nodes_expanded, ms);
        } else {
            std::printf("%-10s %10s\n", "A*-8", "no path");
        }
    }

    // RRT
    {
        RRTPlanner planner(grid);
        RRTConfig cfg;
        cfg.step_size = 3.0;
        cfg.max_iterations = 100000;
        cfg.goal_tolerance = 3.0;
        planner.set_config(cfg);

        auto t0 = std::chrono::high_resolution_clock::now();
        auto r = planner.plan(start, goal);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (r.found) {
            std::printf("%-10s %10.1f %10zu %12d %10.2f\n",
                        "RRT", r.cost, r.path.size(), r.tree_size, ms);
        } else {
            std::printf("%-10s %10s (tree: %d nodes)\n",
                        "RRT", "no path", r.tree_size);
        }
    }

    return 0;
}
