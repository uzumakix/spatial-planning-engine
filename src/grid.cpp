#include "grid.h"
#include <fstream>
#include <iostream>
#include <cmath>

Grid::Grid(int w, int h) : w_(w), h_(h), cells_(w * h, CellState::FREE) {}

bool Grid::load(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "grid: cannot open " << path << "\n";
        return false;
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty())
            lines.push_back(line);
    }

    if (lines.empty()) {
        std::cerr << "grid: empty file\n";
        return false;
    }

    h_ = static_cast<int>(lines.size());
    w_ = static_cast<int>(lines[0].size());
    cells_.resize(w_ * h_, CellState::FREE);

    for (int y = 0; y < h_; ++y) {
        for (int x = 0; x < w_ && x < static_cast<int>(lines[y].size()); ++x) {
            char c = lines[y][x];
            if (c == '#')
                cells_[idx(x, y)] = CellState::BLOCKED;
            else if (c == 'D')
                cells_[idx(x, y)] = CellState::DYNAMIC;
            else
                cells_[idx(x, y)] = CellState::FREE;
        }
    }
    return true;
}

CellState Grid::at(int x, int y) const {
    if (!in_bounds(x, y)) return CellState::BLOCKED;
    return cells_[idx(x, y)];
}

void Grid::set(int x, int y, CellState s) {
    if (in_bounds(x, y))
        cells_[idx(x, y)] = s;
}

bool Grid::in_bounds(int x, int y) const {
    return x >= 0 && x < w_ && y >= 0 && y < h_;
}

bool Grid::passable(int x, int y) const {
    return in_bounds(x, y) && cells_[idx(x, y)] == CellState::FREE;
}

std::vector<Coord> Grid::neighbors(int x, int y, bool eight_connected) const {
    // 4-connected: N, S, E, W
    static const int dx4[] = {1, -1, 0, 0};
    static const int dy4[] = {0, 0, 1, -1};
    // 8-connected adds diagonals
    static const int dx8[] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dy8[] = {0, 0, 1, -1, 1, -1, 1, -1};

    const int* dx = eight_connected ? dx8 : dx4;
    const int* dy = eight_connected ? dy8 : dy4;
    int count = eight_connected ? 8 : 4;

    std::vector<Coord> result;
    result.reserve(count);

    for (int i = 0; i < count; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (passable(nx, ny)) {
            // For diagonal movement, both cardinal neighbors must be free
            // to prevent corner-cutting through walls
            if (eight_connected && i >= 4) {
                if (!passable(x + dx[i], y) || !passable(x, y + dy[i]))
                    continue;
            }
            result.push_back({nx, ny});
        }
    }
    return result;
}

double Grid::cost(const Coord& a, const Coord& b) {
    int dx = std::abs(a.x - b.x);
    int dy = std::abs(a.y - b.y);
    if (dx + dy == 1) return 1.0;
    if (dx == 1 && dy == 1) return 1.41421356237;
    // non-adjacent, return euclidean
    return std::sqrt(dx * dx + dy * dy);
}
