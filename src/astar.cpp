#include "astar.h"
#include <cmath>
#include <algorithm>
#include <limits>

// --- MinHeap implementation ---

void AStarPlanner::MinHeap::init(int grid_size) {
    data.clear();
    data.reserve(256);
    pos.assign(grid_size, -1);
}

void AStarPlanner::MinHeap::push(int index, double f) {
    int i = static_cast<int>(data.size());
    data.push_back({index, f});
    pos[index] = i;
    sift_up(i);
}

auto AStarPlanner::MinHeap::pop() -> HeapNode {
    HeapNode top = data[0];
    pos[top.index] = -1;
    int last = static_cast<int>(data.size()) - 1;
    if (last > 0) {
        swap_nodes(0, last);
        data.pop_back();
        sift_down(0);
    } else {
        data.pop_back();
    }
    return top;
}

void AStarPlanner::MinHeap::decrease(int index, double f) {
    int i = pos[index];
    data[i].f = f;
    sift_up(i);
}

bool AStarPlanner::MinHeap::contains(int index) const {
    return index >= 0 && index < static_cast<int>(pos.size()) && pos[index] != -1;
}

void AStarPlanner::MinHeap::sift_up(int i) {
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (data[i].f < data[parent].f) {
            swap_nodes(i, parent);
            i = parent;
        } else break;
    }
}

void AStarPlanner::MinHeap::sift_down(int i) {
    int n = static_cast<int>(data.size());
    while (true) {
        int smallest = i;
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        if (left < n && data[left].f < data[smallest].f)
            smallest = left;
        if (right < n && data[right].f < data[smallest].f)
            smallest = right;
        if (smallest == i) break;
        swap_nodes(i, smallest);
        i = smallest;
    }
}

void AStarPlanner::MinHeap::swap_nodes(int i, int j) {
    pos[data[i].index] = j;
    pos[data[j].index] = i;
    std::swap(data[i], data[j]);
}

// --- AStarPlanner ---

AStarPlanner::AStarPlanner(const Grid& grid, bool eight_conn)
    : grid_(grid), eight_conn_(eight_conn) {}

double AStarPlanner::h(const Coord& a, const Coord& b) const {
    double dx = std::abs(a.x - b.x);
    double dy = std::abs(a.y - b.y);
    if (heuristic_ == Heuristic::MANHATTAN)
        return dx + dy;
    return std::sqrt(dx * dx + dy * dy);
}

PlanResult AStarPlanner::plan(Coord start, Coord goal) {
    PlanResult result{{}, 0.0, 0, false};

    if (!grid_.passable(start.x, start.y) || !grid_.passable(goal.x, goal.y))
        return result;

    if (start == goal) {
        result.path.push_back(start);
        result.found = true;
        return result;
    }

    int grid_size = grid_.width() * grid_.height();
    int w = grid_.width();

    auto lin = [w](int x, int y) { return y * w + x; };
    auto coord_of = [w](int idx) -> Coord { return {idx % w, idx / w}; };

    std::vector<double> g(grid_size, std::numeric_limits<double>::infinity());
    std::vector<int> parent(grid_size, -1);
    std::vector<bool> closed(grid_size, false);

    MinHeap open;
    open.init(grid_size);

    int si = lin(start.x, start.y);
    g[si] = 0.0;
    open.push(si, h(start, goal));

    while (!open.empty()) {
        auto current = open.pop();
        int ci = current.index;
        Coord cc = coord_of(ci);

        if (cc == goal) {
            result.found = true;
            result.cost = g[ci];
            // reconstruct path
            int idx = ci;
            while (idx != -1) {
                result.path.push_back(coord_of(idx));
                idx = parent[idx];
            }
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }

        closed[ci] = true;
        result.nodes_expanded++;

        auto nbrs = grid_.neighbors(cc.x, cc.y, eight_conn_);
        for (const auto& nb : nbrs) {
            int ni = lin(nb.x, nb.y);
            if (closed[ni]) continue;

            double tentative = g[ci] + Grid::cost(cc, nb);
            if (tentative < g[ni]) {
                g[ni] = tentative;
                parent[ni] = ci;
                double f = tentative + h(nb, goal);
                if (open.contains(ni))
                    open.decrease(ni, f);
                else
                    open.push(ni, f);
            }
        }
    }

    return result;  // no path
}
