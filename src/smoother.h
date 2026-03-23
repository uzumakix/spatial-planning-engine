#ifndef SPATIAL_SMOOTHER_H
#define SPATIAL_SMOOTHER_H

#include "grid.h"
#include <vector>

struct SmoothResult {
    std::vector<Coord> path;
    double reduction_pct;  // how much shorter vs original (by waypoint count)
};

class PathSmoother {
public:
    explicit PathSmoother(const Grid& grid);

    // Iterative shortcutting. Tries to skip intermediate waypoints
    // when the direct connection is collision-free.
    // max_iters controls how many random shortcut attempts to make.
    SmoothResult smooth(const std::vector<Coord>& path, int max_iters = 200);

private:
    const Grid& grid_;

    bool line_clear(Coord a, Coord b) const;
};

#endif
