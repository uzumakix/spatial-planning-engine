#include "smoother.h"
#include <cmath>
#include <random>

PathSmoother::PathSmoother(const Grid& grid) : grid_(grid) {}

SmoothResult PathSmoother::smooth(const std::vector<Coord>& path, int max_iters) {
    if (path.size() <= 2)
        return {path, 0.0};

    std::vector<Coord> smoothed = path;
    std::mt19937 rng(12345);
    int original_size = static_cast<int>(path.size());

    for (int iter = 0; iter < max_iters && smoothed.size() > 2; ++iter) {
        int n = static_cast<int>(smoothed.size());
        std::uniform_int_distribution<int> dist_i(0, n - 2);
        int i = dist_i(rng);
        std::uniform_int_distribution<int> dist_j(i + 2, n - 1);
        int j = dist_j(rng);

        if (line_clear(smoothed[i], smoothed[j])) {
            // remove intermediate waypoints between i and j
            smoothed.erase(smoothed.begin() + i + 1, smoothed.begin() + j);
        }
    }

    double reduction = 100.0 * (1.0 - static_cast<double>(smoothed.size()) / original_size);
    return {std::move(smoothed), reduction};
}

bool PathSmoother::line_clear(Coord a, Coord b) const {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double d = std::sqrt(dx * dx + dy * dy);
    int steps = static_cast<int>(std::ceil(d / 0.5));
    if (steps == 0) return true;

    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        int cx = static_cast<int>(std::round(a.x + t * dx));
        int cy = static_cast<int>(std::round(a.y + t * dy));
        if (!grid_.passable(cx, cy))
            return false;
    }
    return true;
}
