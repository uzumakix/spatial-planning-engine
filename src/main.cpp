#include "grid.h"
#include "astar.h"
#include "rrt.h"
#include "smoother.h"
#include <iostream>
#include <string>
#include <chrono>
#include <cstdlib>

static void usage() {
    std::cerr << "usage: planner --grid <file> --start x,y --goal x,y "
              << "--algo astar|rrt [--smooth] [--eight]\n";
    std::exit(1);
}

static Coord parse_coord(const std::string& s) {
    auto comma = s.find(',');
    if (comma == std::string::npos) {
        std::cerr << "bad coord: " << s << "\n";
        std::exit(1);
    }
    return {std::stoi(s.substr(0, comma)),
            std::stoi(s.substr(comma + 1))};
}

int main(int argc, char* argv[]) {
    std::string grid_path, algo;
    Coord start{-1, -1}, goal{-1, -1};
    bool do_smooth = false;
    bool eight_conn = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--grid" && i + 1 < argc)
            grid_path = argv[++i];
        else if (arg == "--start" && i + 1 < argc)
            start = parse_coord(argv[++i]);
        else if (arg == "--goal" && i + 1 < argc)
            goal = parse_coord(argv[++i]);
        else if (arg == "--algo" && i + 1 < argc)
            algo = argv[++i];
        else if (arg == "--smooth")
            do_smooth = true;
        else if (arg == "--eight")
            eight_conn = true;
        else
            usage();
    }

    if (grid_path.empty() || algo.empty() || start.x < 0 || goal.x < 0)
        usage();

    Grid grid;
    if (!grid.load(grid_path)) return 1;

    std::cout << "grid: " << grid.width() << "x" << grid.height() << "\n";
    std::cout << "start: (" << start.x << "," << start.y << ") "
              << "goal: (" << goal.x << "," << goal.y << ")\n";

    auto t0 = std::chrono::high_resolution_clock::now();

    if (algo == "astar") {
        AStarPlanner planner(grid, eight_conn);
        auto result = planner.plan(start, goal);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (!result.found) {
            std::cout << "no path found\n";
            return 1;
        }

        std::cout << "algo: A*\n";
        std::cout << "path length: " << result.path.size() << " waypoints\n";
        std::cout << "cost: " << result.cost << "\n";
        std::cout << "nodes expanded: " << result.nodes_expanded << "\n";
        std::cout << "time: " << ms << " ms\n";

        if (do_smooth) {
            PathSmoother smoother(grid);
            auto sr = smoother.smooth(result.path);
            std::cout << "smoothed: " << sr.path.size()
                      << " waypoints (" << sr.reduction_pct << "% reduction)\n";
            result.path = std::move(sr.path);
        }

        // print path
        std::cout << "path:";
        for (const auto& p : result.path)
            std::cout << " (" << p.x << "," << p.y << ")";
        std::cout << "\n";

    } else if (algo == "rrt") {
        RRTPlanner planner(grid);
        auto result = planner.plan(start, goal);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (!result.found) {
            std::cout << "no path found after " << result.iterations_used << " iterations\n";
            return 1;
        }

        std::cout << "algo: RRT\n";
        std::cout << "path length: " << result.path.size() << " waypoints\n";
        std::cout << "cost: " << result.cost << "\n";
        std::cout << "tree size: " << result.tree_size << " nodes\n";
        std::cout << "iterations: " << result.iterations_used << "\n";
        std::cout << "time: " << ms << " ms\n";

        if (do_smooth) {
            PathSmoother smoother(grid);
            auto sr = smoother.smooth(result.path);
            std::cout << "smoothed: " << sr.path.size()
                      << " waypoints (" << sr.reduction_pct << "% reduction)\n";
            result.path = std::move(sr.path);
        }

        std::cout << "path:";
        for (const auto& p : result.path)
            std::cout << " (" << p.x << "," << p.y << ")";
        std::cout << "\n";

    } else {
        std::cerr << "unknown algo: " << algo << "\n";
        return 1;
    }

    return 0;
}
