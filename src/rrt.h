#ifndef SPATIAL_RRT_H
#define SPATIAL_RRT_H

#include "grid.h"
#include <vector>
#include <random>

struct RRTConfig {
    double step_size = 1.5;
    double goal_bias = 0.05;   // probability of sampling the goal
    int max_iterations = 50000;
    double goal_tolerance = 1.5;
};

struct RRTResult {
    std::vector<Coord> path;
    double cost;
    int tree_size;
    int iterations_used;
    bool found;
};

class RRTPlanner {
public:
    RRTPlanner(const Grid& grid, unsigned seed = 42);

    void set_config(const RRTConfig& cfg) { cfg_ = cfg; }

    RRTResult plan(Coord start, Coord goal);

private:
    const Grid& grid_;
    RRTConfig cfg_;
    std::mt19937 rng_;

    struct Node {
        double x, y;
        int parent;  // index into nodes_
    };
    std::vector<Node> nodes_;

    // Sample a random point in the grid. With probability goal_bias, returns goal.
    std::pair<double, double> sample(Coord goal);

    // Find index of nearest node to (px, py)
    int nearest(double px, double py) const;

    // Steer from (fx, fy) toward (tx, ty) by at most step_size
    std::pair<double, double> steer(double fx, double fy, double tx, double ty) const;

    // Check if the segment from (x0,y0) to (x1,y1) is collision-free.
    // Uses Bresenham-style sampling.
    bool collision_free(double x0, double y0, double x1, double y1) const;

    double dist(double x0, double y0, double x1, double y1) const;
};

#endif
