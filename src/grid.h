#ifndef SPATIAL_GRID_H
#define SPATIAL_GRID_H

#include <cstdint>
#include <string>
#include <vector>
#include <utility>

enum class CellState : uint8_t {
    FREE = 0,
    BLOCKED = 1,
    DYNAMIC = 2
};

struct Coord {
    int x, y;
    bool operator==(const Coord& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Coord& o) const { return !(*this == o); }
};

class Grid {
public:
    Grid() = default;
    Grid(int w, int h);

    // Load from text file. Returns false on failure.
    bool load(const std::string& path);

    int width() const { return w_; }
    int height() const { return h_; }

    CellState at(int x, int y) const;
    void set(int x, int y, CellState s);
    bool in_bounds(int x, int y) const;
    bool passable(int x, int y) const;

    // Neighbor queries. eight_connected includes diagonals.
    std::vector<Coord> neighbors(int x, int y, bool eight_connected = false) const;

    // Movement cost between adjacent cells. Diagonal = sqrt(2).
    static double cost(const Coord& a, const Coord& b);

private:
    int w_ = 0, h_ = 0;
    std::vector<CellState> cells_;

    int idx(int x, int y) const { return y * w_ + x; }
};

#endif
