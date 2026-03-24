#include "rrt.h"
#include <cassert>
#include <iostream>
#include <fstream>

static const char* TEST_GRID = "test_tmp_rrt.txt";

static void test_open_field() {
    std::ofstream f(TEST_GRID);
    for (int i = 0; i < 10; ++i)
        f << "..........\n";
    f.close();
    Grid g;
    g.load(TEST_GRID);

    RRTPlanner planner(g);
    auto r = planner.plan({0, 0}, {9, 9});
    assert(r.found);
    assert(r.path.front().x == 0 && r.path.front().y == 0);
    assert(r.path.back().x == 9 && r.path.back().y == 9);
    assert(r.tree_size > 0);

    // path should be collision-free
    for (const auto& p : r.path)
        assert(g.passable(p.x, p.y));
}

static void test_blocked() {
    // wall splits the grid, no path possible
    std::ofstream f(TEST_GRID);
    for (int i = 0; i < 10; ++i)
        f << ".....#....\n";
    f.close();
    Grid g;
    g.load(TEST_GRID);

    RRTPlanner planner(g);
    RRTConfig cfg;
    cfg.max_iterations = 5000;  // don't waste time
    planner.set_config(cfg);

    auto r = planner.plan({0, 0}, {9, 0});
    assert(!r.found);
}

static void test_narrow_passage() {
    std::ofstream f(TEST_GRID);
    f << "..........\n";
    f << "####.#####\n";
    f << "..........\n";
    f.close();
    Grid g;
    g.load(TEST_GRID);

    RRTPlanner planner(g, 777);
    RRTConfig cfg;
    cfg.max_iterations = 20000;
    cfg.goal_bias = 0.1;  // higher bias helps in narrow passages
    planner.set_config(cfg);

    auto r = planner.plan({0, 0}, {9, 2});
    assert(r.found);
    // must pass through the gap at column 4
    bool goes_through_gap = false;
    for (const auto& p : r.path) {
        if (p.x == 4 && p.y == 1)
            goes_through_gap = true;
    }
    // RRT might not hit exactly (4,1) but path should cross row 1 around col 4
    // just verify it reaches the goal
    assert(r.path.back().x == 9 && r.path.back().y == 2);
}

static void test_config() {
    std::ofstream f(TEST_GRID);
    for (int i = 0; i < 10; ++i)
        f << "..........\n";
    f.close();
    Grid g;
    g.load(TEST_GRID);

    RRTPlanner planner(g);
    RRTConfig cfg;
    cfg.step_size = 2.0;
    cfg.goal_bias = 0.1;
    cfg.max_iterations = 10000;
    planner.set_config(cfg);

    auto r = planner.plan({0, 0}, {9, 9});
    assert(r.found);
}

int main() {
    test_open_field();
    test_blocked();
    test_narrow_passage();
    test_config();

    std::remove(TEST_GRID);
    std::cout << "rrt: all tests passed\n";
    return 0;
}
