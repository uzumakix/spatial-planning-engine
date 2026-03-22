#ifndef SPATIAL_ASTAR_H
#define SPATIAL_ASTAR_H

#include "grid.h"
#include <vector>
#include <functional>

enum class Heuristic { MANHATTAN, EUCLIDEAN };

struct PlanResult {
    std::vector<Coord> path;
    double cost;
    int nodes_expanded;
    bool found;
};

class AStarPlanner {
public:
    explicit AStarPlanner(const Grid& grid, bool eight_conn = false);

    void set_heuristic(Heuristic h) { heuristic_ = h; }

    PlanResult plan(Coord start, Coord goal);

    // Min-heap for the open set. Hand-rolled because std::priority_queue
    // doesn't support decrease-key, and the overhead matters on big grids.
    struct HeapNode {
        int index;  // linearized grid index
        double f;
    };

    struct MinHeap {
        std::vector<HeapNode> data;
        std::vector<int> pos;  // grid index -> position in heap (-1 if absent)

        void init(int grid_size);
        void push(int index, double f);
        HeapNode pop();
        void decrease(int index, double f);
        bool contains(int index) const;
        bool empty() const { return data.empty(); }

    private:
        void sift_up(int i);
        void sift_down(int i);
        void swap_nodes(int i, int j);
    };

private:
    const Grid& grid_;
    bool eight_conn_;
    Heuristic heuristic_ = Heuristic::EUCLIDEAN;

    double h(const Coord& a, const Coord& b) const;
};

#endif
