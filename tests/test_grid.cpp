// test_grid.cpp - Unit tests for Grid class.
#include "core/Grid.hpp"
#include <cassert>
#include <iostream>
#include <string>

namespace {

int passed = 0, failed = 0;

void test(bool cond, const std::string& name) {
    if (cond) { std::cout << "  [PASS] " << name << "\n"; ++passed; }
    else       { std::cout << "  [FAIL] " << name << "\n"; ++failed; }
}

void test_initialization() {
    warehouse::Grid g(10, 8);
    test(g.cols() == 10,     "Grid: cols == 10");
    test(g.rows() == 8,      "Grid: rows == 8");
    test(g.obstacleCount() == 0, "Grid: initially no obstacles");
}

void test_bounds() {
    warehouse::Grid g(5, 5);
    test(g.inBounds(0, 0),   "Grid: (0,0) in bounds");
    test(g.inBounds(4, 4),   "Grid: (4,4) in bounds");
    test(!g.inBounds(-1, 0), "Grid: (-1,0) out of bounds");
    test(!g.inBounds(5, 0),  "Grid: (5,0) out of bounds");
    test(!g.inBounds(0, 5),  "Grid: (0,5) out of bounds");
    test(g.isObstacle(10,10),"Grid: out-of-bounds treated as obstacle");
}

void test_obstacle_management() {
    warehouse::Grid g(5, 5);
    g.setObstacle(2, 2);
    test(g.isObstacle(2, 2),    "Grid: setObstacle works");
    test(!g.isObstacle(0, 0),   "Grid: non-obstacle cell is free");
    test(g.obstacleCount() == 1,"Grid: obstacle count = 1");

    g.setObstacle(2, 2, false);
    test(!g.isObstacle(2, 2),   "Grid: clearObstacle works");
    test(g.obstacleCount() == 0,"Grid: obstacle count = 0 after clear");

    // Removed obsolete toggling test
}

void test_clear() {
    warehouse::Grid g(5, 5);
    g.setObstacle(0, 0); g.setObstacle(1, 1); g.setObstacle(2, 2);
    test(g.obstacleCount() == 3, "Grid: 3 obstacles before clear");
    g.clear();
    test(g.obstacleCount() == 0, "Grid: clear removes all obstacles");
}

void test_neighbours() {
    warehouse::Grid g(5, 5);
    auto ns = g.neighbours(2, 2);
    test(ns.size() == 4, "Grid: centre cell has 4 neighbours");

    auto corner_ns = g.neighbours(0, 0);
    test(corner_ns.size() == 2, "Grid: corner has 2 neighbours");

    g.setObstacle(2, 1);
    auto ns2 = g.neighbours(2, 2);
    test(ns2.size() == 3, "Grid: obstacle reduces neighbour count");
}

void test_default_map() {
    warehouse::Grid g(30, 22);
    g.generateRealisticWarehouse();
    test(g.obstacleCount() > 0, "Grid: default map has obstacles");
}

void test_random_generation() {
    warehouse::Grid g(20, 20);
    g.generateRandom(0.2f, {{0,0}, {19,19}});
    test(g.isFree(0, 0),   "Grid: random map protects (0,0)");
    test(g.isFree(19, 19), "Grid: random map protects (19,19)");
    test(g.obstacleCount() > 0, "Grid: random map has obstacles");
}

void test_free_cells() {
    warehouse::Grid g(5, 5);
    g.setObstacle(0, 0); g.setObstacle(1, 1);
    auto free = g.freeCells();
    test(static_cast<int>(free.size()) == 23, "Grid: freeCells count = 23");
}

}  // anonymous namespace

int main() {
    std::cout << "=== Grid Unit Tests ===\n";
    test_initialization();
    test_bounds();
    test_obstacle_management();
    test_clear();
    test_neighbours();
    test_default_map();
    test_random_generation();
    test_free_cells();

    std::cout << "\n=== Results: " << passed << " passed, " << failed << " failed ===\n";
    return (failed == 0) ? 0 : 1;
}
