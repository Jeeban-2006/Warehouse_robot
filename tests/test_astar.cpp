// test_astar.cpp - Unit tests for AStarPlanner.
// Uses simple assertion-based testing. No external framework required.
#include "core/Grid.hpp"
#include "core/AStarPlanner.hpp"
#include <cassert>
#include <iostream>
#include <string>

namespace {

int passed = 0, failed = 0;

void test(bool cond, const std::string& name) {
    if (cond) {
        std::cout << "  [PASS] " << name << "\n";
        ++passed;
    } else {
        std::cout << "  [FAIL] " << name << "\n";
        ++failed;
    }
}

void TEST_01_simple_path() {
    warehouse::Grid g(10, 10);
    warehouse::AStarPlanner pf(g);
    auto r = pf.findPath({0,0}, {9,9});
    test(r.success, "TEST_01: simple path found on empty grid");
    test(r.pathLength > 0, "TEST_01: path length > 0");
    test(r.path.front() == std::make_pair(0,0), "TEST_01: path starts at start");
    test(r.path.back()  == std::make_pair(9,9), "TEST_01: path ends at goal");
}

void TEST_02_single_obstacle() {
    warehouse::Grid g(5, 5);
    g.setObstacle(2, 0);
    warehouse::AStarPlanner pf(g);
    auto r = pf.findPath({0,0}, {4,0});
    test(r.success, "TEST_02: path found around single obstacle");
}

void TEST_03_wall_of_obstacles() {
    warehouse::Grid g(10, 5);
    for (int r = 0; r < 5; ++r) g.setObstacle(5, r);  // vertical wall
    g.setObstacle(5, 3, false);                          // gap in wall
    warehouse::AStarPlanner pf(g);
    auto r = pf.findPath({0,0}, {9,0});
    test(r.success, "TEST_03: path found through gap in wall");
}

void TEST_04_multiple_obstacles() {
    warehouse::Grid g(10, 10);
    for (int c : {2,3,4,5,6}) g.setObstacle(c, 5);
    for (int c : {2,3,4,5,6}) g.setObstacle(c, 2);
    warehouse::AStarPlanner pf(g);
    auto r = pf.findPath({0,0}, {9,9});
    test(r.success, "TEST_04: path found with multiple obstacles");
}

void TEST_05_unreachable_goal() {
    warehouse::Grid g(5, 5);
    // Completely surround (2,2)
    g.setObstacle(1,2); g.setObstacle(3,2);
    g.setObstacle(2,1); g.setObstacle(2,3);
    warehouse::AStarPlanner pf(g);
    auto r = pf.findPath({0,0}, {2,2});
    test(!r.success, "TEST_05: unreachable goal returns failure");
    test(!r.message.empty(), "TEST_05: failure message is set");
}

void TEST_06_start_equals_goal() {
    warehouse::Grid g(5,5);
    warehouse::AStarPlanner pf(g);
    auto r = pf.findPath({2,2}, {2,2});
    test(r.success, "TEST_06: start==goal succeeds");
    test(r.pathLength == 1, "TEST_06: path length is 1");
}

void TEST_07_start_blocked() {
    warehouse::Grid g(5,5);
    g.setObstacle(0,0);
    warehouse::AStarPlanner pf(g);
    auto r = pf.findPath({0,0}, {4,4});
    test(!r.success, "TEST_07: start blocked returns failure");
}

void TEST_08_goal_blocked() {
    warehouse::Grid g(5,5);
    g.setObstacle(4,4);
    warehouse::AStarPlanner pf(g);
    auto r = pf.findPath({0,0}, {4,4});
    test(!r.success, "TEST_08: goal blocked returns failure");
}

void TEST_09_extra_blocked() {
    warehouse::Grid g(10,10);
    warehouse::AStarPlanner pf(g);
    // Block all cells in column 5 via extraBlocked
    warehouse::PosSet extra;
    for (int r = 0; r < 10; ++r) extra.insert({5,r});
    extra.erase({5,5});   // leave gap
    auto r = pf.findPath({0,5}, {9,5}, extra);
    test(r.success, "TEST_09: path found using extraBlocked set");
}

void TEST_10_path_validity() {
    warehouse::Grid g(10,10);
    warehouse::AStarPlanner pf(g);
    auto r = pf.findPath({0,0}, {9,9});
    test(r.success, "TEST_10: initial path found");
    test(pf.isPathValid(r.path), "TEST_10: path valid on clean grid");
    g.setObstacle(1,1);   // block a cell on the path
    test(!pf.isPathValid(r.path), "TEST_10: path invalid after obstacle placed");
}

void TEST_11_out_of_bounds_start() {
    warehouse::Grid g(5,5);
    warehouse::AStarPlanner pf(g);
    auto r = pf.findPath({-1,0}, {4,4});
    test(!r.success, "TEST_11: out-of-bounds start returns failure");
}

void TEST_12_out_of_bounds_goal() {
    warehouse::Grid g(5,5);
    warehouse::AStarPlanner pf(g);
    auto r = pf.findPath({0,0}, {10,10});
    test(!r.success, "TEST_12: out-of-bounds goal returns failure");
}

void TEST_13_manhattan_optimality() {
    // On an empty grid, Manhattan distance should equal path length-1
    warehouse::Grid g(10,10);
    warehouse::AStarPlanner pf(g);
    auto r = pf.findPath({0,0}, {5,3});
    int expected = 5 + 3;  // Manhattan distance
    test(r.pathLength == expected + 1, "TEST_13: path length == Manhattan distance + 1");
}

void TEST_14_explored_populated() {
    warehouse::Grid g(10,10);
    warehouse::AStarPlanner pf(g);
    auto r = pf.findPath({0,0}, {9,9});
    test(r.nodesExplored > 0, "TEST_14: explored nodes > 0");
    test(r.planningTimeMs >= 0.0, "TEST_14: planning time >= 0");
}

void TEST_15_statistics_cumulative() {
    warehouse::Grid g(10,10);
    warehouse::AStarPlanner pf(g);
    pf.findPath({0,0},{5,5});
    pf.findPath({0,0},{9,9});
    test(pf.stats().totalPlanCalls == 2, "TEST_15: stats count 2 calls");
}

}  // anonymous namespace

int main() {
    std::cout << "=== AStarPlanner Unit Tests ===\n";
    TEST_01_simple_path();
    TEST_02_single_obstacle();
    TEST_03_wall_of_obstacles();
    TEST_04_multiple_obstacles();
    TEST_05_unreachable_goal();
    TEST_06_start_equals_goal();
    TEST_07_start_blocked();
    TEST_08_goal_blocked();
    TEST_09_extra_blocked();
    TEST_10_path_validity();
    TEST_11_out_of_bounds_start();
    TEST_12_out_of_bounds_goal();
    TEST_13_manhattan_optimality();
    TEST_14_explored_populated();
    TEST_15_statistics_cumulative();

    std::cout << "\n=== Results: " << passed << " passed, " << failed << " failed ===\n";
    return (failed == 0) ? 0 : 1;
}
