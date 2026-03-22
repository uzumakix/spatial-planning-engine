#include "rrt.h"
#include <cmath>
#include <algorithm>

RRTPlanner::RRTPlanner(const Grid& grid, unsigned seed)
    : grid_(grid), rng_(seed) {}

RRTResult RRTPlanner::plan(Coord start, Coord goal) {
    RRTResult result{{}, 0.0, 0, 0, false};

    if (!grid_.passable(start.x, start.y) || !grid_.passable(goal.x, goal.y))
        return result;

    nodes_.clear();
    nodes_.reserve(cfg_.max_iterations / 4);
    nodes_.push_back({static_cast<double>(start.x),
                      static_cast<double>(start.y), -1});

    for (int iter = 0; iter < cfg_.max_iterations; ++iter) {
        auto [sx, sy] = sample(goal);
        int ni = nearest(sx, sy);
        auto [nx, ny] = steer(nodes_[ni].x, nodes_[ni].y, sx, sy);

        if (!collision_free(nodes_[ni].x, nodes_[ni].y, nx, ny))
            continue;

        // snap to integer coords for grid consistency
        int ix = static_cast<int>(std::round(nx));
        int iy = static_cast<int>(std::round(ny));
        if (!grid_.passable(ix, iy))
            continue;

        int new_idx = static_cast<int>(nodes_.size());
        nodes_.push_back({nx, ny, ni});

        // check if we reached the goal
        if (dist(nx, ny, goal.x, goal.y) <= cfg_.goal_tolerance) {
            result.found = true;
            result.iterations_used = iter + 1;
            result.tree_size = static_cast<int>(nodes_.size());

            // trace back
            std::vector<Coord> path;
            int idx = new_idx;
            while (idx != -1) {
                path.push_back({static_cast<int>(std::round(nodes_[idx].x)),
                                static_cast<int>(std::round(nodes_[idx].y))});
                idx = nodes_[idx].parent;
            }
            std::reverse(path.begin(), path.end());

            // add the exact goal if not already there
            if (path.back() != goal)
                path.push_back(goal);

            // compute path cost
            double total = 0.0;
            for (size_t i = 1; i < path.size(); ++i)
                total += Grid::cost(path[i - 1], path[i]);

            result.path = std::move(path);
            result.cost = total;
            return result;
        }
    }

    result.tree_size = static_cast<int>(nodes_.size());
    result.iterations_used = cfg_.max_iterations;
    return result;
}

std::pair<double, double> RRTPlanner::sample(Coord goal) {
    std::uniform_real_distribution<double> coin(0.0, 1.0);
    if (coin(rng_) < cfg_.goal_bias)
        return {static_cast<double>(goal.x), static_cast<double>(goal.y)};

    std::uniform_real_distribution<double> xdist(0.0, grid_.width() - 1.0);
    std::uniform_real_distribution<double> ydist(0.0, grid_.height() - 1.0);
    return {xdist(rng_), ydist(rng_)};
}

int RRTPlanner::nearest(double px, double py) const {
    // brute force, fine for grids under ~1000x1000
    // TODO: k-d tree if this becomes a bottleneck for the swarm coordinator
    int best = 0;
    double best_d = dist(nodes_[0].x, nodes_[0].y, px, py);
    for (int i = 1; i < static_cast<int>(nodes_.size()); ++i) {
        double d = dist(nodes_[i].x, nodes_[i].y, px, py);
        if (d < best_d) {
            best_d = d;
            best = i;
        }
    }
    return best;
}

std::pair<double, double> RRTPlanner::steer(double fx, double fy,
                                             double tx, double ty) const {
    double d = dist(fx, fy, tx, ty);
    if (d <= cfg_.step_size)
        return {tx, ty};
    double ratio = cfg_.step_size / d;
    return {fx + ratio * (tx - fx), fy + ratio * (ty - fy)};
}

bool RRTPlanner::collision_free(double x0, double y0,
                                 double x1, double y1) const {
    double d = dist(x0, y0, x1, y1);
    int steps = static_cast<int>(std::ceil(d / 0.5));
    if (steps == 0) steps = 1;

    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        int cx = static_cast<int>(std::round(x0 + t * (x1 - x0)));
        int cy = static_cast<int>(std::round(y0 + t * (y1 - y0)));
        if (!grid_.passable(cx, cy))
            return false;
    }
    return true;
}

double RRTPlanner::dist(double x0, double y0, double x1, double y1) const {
    double dx = x1 - x0;
    double dy = y1 - y0;
    return std::sqrt(dx * dx + dy * dy);
}
