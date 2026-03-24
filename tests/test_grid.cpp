#include "grid.h"
#include <cassert>
#include <iostream>
#include <fstream>

static const char* TEST_GRID = "test_tmp_grid.txt";

static void write_test_grid() {
    std::ofstream f(TEST_GRID);
    // 5x4 grid
    f << ".....\n";
    f << ".###.\n";
    f << ".#...\n";
    f << ".....\n";
}

static void test_load() {
    write_test_grid();
    Grid g;
    assert(g.load(TEST_GRID));
    assert(g.width() == 5);
    assert(g.height() == 4);
}

static void test_cell_states() {
    write_test_grid();
    Grid g;
    g.load(TEST_GRID);

    assert(g.at(0, 0) == CellState::FREE);
    assert(g.at(1, 1) == CellState::BLOCKED);
    assert(g.at(2, 1) == CellState::BLOCKED);
    assert(g.at(3, 1) == CellState::BLOCKED);
    assert(g.at(4, 1) == CellState::FREE);
    assert(g.at(1, 2) == CellState::BLOCKED);
    assert(g.at(2, 2) == CellState::FREE);
}

static void test_bounds() {
    Grid g(10, 10);
    assert(g.in_bounds(0, 0));
    assert(g.in_bounds(9, 9));
    assert(!g.in_bounds(-1, 0));
    assert(!g.in_bounds(10, 0));
    assert(!g.in_bounds(0, 10));
}

static void test_neighbors_4() {
    write_test_grid();
    Grid g;
    g.load(TEST_GRID);

    // corner: (0,0) should have 2 free neighbors
    auto n = g.neighbors(0, 0, false);
    assert(n.size() == 2);  // right and down

    // (0,1) has blocked to the right, so only up and down
    n = g.neighbors(0, 1, false);
    assert(n.size() == 2);
}

static void test_neighbors_8() {
    Grid g(5, 5);  // all free
    auto n = g.neighbors(2, 2, true);
    assert(n.size() == 8);

    // corner
    n = g.neighbors(0, 0, true);
    assert(n.size() == 3);
}

static void test_passable() {
    write_test_grid();
    Grid g;
    g.load(TEST_GRID);

    assert(g.passable(0, 0));
    assert(!g.passable(1, 1));
    assert(!g.passable(-1, -1));  // out of bounds
}

static void test_cost() {
    Coord a{0, 0}, b{1, 0}, c{1, 1};
    assert(Grid::cost(a, b) == 1.0);
    double diag = Grid::cost(a, c);
    assert(diag > 1.41 && diag < 1.42);
}

static void test_dynamic_cell() {
    std::ofstream f(TEST_GRID);
    f << "..D\n";
    f << ".#.\n";
    f.close();

    Grid g;
    g.load(TEST_GRID);
    // dynamic obstacles are loaded but treated as non-passable
    // (they block like regular obstacles until cleared)
    assert(g.at(2, 0) == CellState::DYNAMIC);
    // dynamic cells are not FREE, so passable returns false
    assert(!g.passable(2, 0));
}

int main() {
    test_load();
    test_cell_states();
    test_bounds();
    test_neighbors_4();
    test_neighbors_8();
    test_passable();
    test_cost();
    test_dynamic_cell();

    std::remove(TEST_GRID);
    std::cout << "grid: all tests passed\n";
    return 0;
}
