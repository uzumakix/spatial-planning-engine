#include "astar.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include <cmath>

static const char* TEST_GRID = "test_tmp_astar.txt";

static void test_straight_line() {
    // 5x1 corridor
    std::ofstream f(TEST_GRID);
    f << ".....\n";
    f.close();
    Grid g;
    g.load(TEST_GRID);

    AStarPlanner planner(g);
    auto r = planner.plan({0, 0}, {4, 0});
    assert(r.found);
    assert(r.path.size() == 5);
    assert(r.cost == 4.0);
    assert(r.path.front().x == 0);
    assert(r.path.back().x == 4);
}

static void test_around_wall() {
    std::ofstream f(TEST_GRID);
    f << ".....\n";
    f << "..#..\n";
    f << "..#..\n";
    f << ".....\n";
    f.close();
    Grid g;
    g.load(TEST_GRID);

    AStarPlanner planner(g);
    auto r = planner.plan({0, 1}, {4, 1});
    assert(r.found);
    // path must go around the wall, cost should be > 4
    assert(r.cost > 4.0);
    // verify no waypoint is on a blocked cell
    for (const auto& p : r.path)
        assert(g.passable(p.x, p.y));
}

static void test_no_path() {
    // completely walled off
    std::ofstream f(TEST_GRID);
    f << "..#..\n";
    f << "..#..\n";
    f << "..#..\n";
    f << "..#..\n";
    f << "..#..\n";
    f.close();
    Grid g;
    g.load(TEST_GRID);

    AStarPlanner planner(g);
    auto r = planner.plan({0, 0}, {4, 0});
    assert(!r.found);
    assert(r.path.empty());
}

static void test_start_equals_goal() {
    std::ofstream f(TEST_GRID);
    f << "...\n...\n";
    f.close();
    Grid g;
    g.load(TEST_GRID);

    AStarPlanner planner(g);
    auto r = planner.plan({1, 1}, {1, 1});
    assert(r.found);
    assert(r.path.size() == 1);
    assert(r.cost == 0.0);
}

static void test_start_on_wall() {
    std::ofstream f(TEST_GRID);
    f << ".#.\n...\n";
    f.close();
    Grid g;
    g.load(TEST_GRID);

    AStarPlanner planner(g);
    auto r = planner.plan({1, 0}, {2, 1});
    assert(!r.found);
}

static void test_manhattan_heuristic() {
    std::ofstream f(TEST_GRID);
    f << ".....\n";
    f << ".....\n";
    f << ".....\n";
    f.close();
    Grid g;
    g.load(TEST_GRID);

    AStarPlanner planner(g);
    planner.set_heuristic(Heuristic::MANHATTAN);
    auto r = planner.plan({0, 0}, {4, 2});
    assert(r.found);
    // manhattan cost for 4-connected: |dx| + |dy| = 6
    assert(std::abs(r.cost - 6.0) < 0.001);
}

static void test_eight_connected() {
    std::ofstream f(TEST_GRID);
    f << "...\n...\n...\n";
    f.close();
    Grid g;
    g.load(TEST_GRID);

    AStarPlanner planner(g, true);
    auto r = planner.plan({0, 0}, {2, 2});
    assert(r.found);
    // diagonal path: 2 diagonal moves, cost = 2*sqrt(2) ~ 2.828
    assert(r.cost < 4.0);  // should be less than manhattan
}

int main() {
    test_straight_line();
    test_around_wall();
    test_no_path();
    test_start_equals_goal();
    test_start_on_wall();
    test_manhattan_heuristic();
    test_eight_connected();

    std::remove(TEST_GRID);
    std::cout << "astar: all tests passed\n";
    return 0;
}
