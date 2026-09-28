// test_robot.cpp - Unit tests for Robot class.
#include "core/Robot.hpp"
#include "core/Grid.hpp"
#include "core/AStarPlanner.hpp"
#include <cassert>
#include <iostream>
#include <string>

namespace {

int passed = 0, failed = 0;

void test(bool cond, const std::string& name) {
    if (cond) { std::cout << "  [PASS] " << name << "\n"; ++passed; }
    else       { std::cout << "  [FAIL] " << name << "\n"; ++failed; }
}

void test_initial_state() {
    warehouse::Robot r(0, {1,1}, {8,8}, 32, 4.0f);
    test(r.col() == 1, "Robot: initial col = 1");
    test(r.row() == 1, "Robot: initial row = 1");
    test(r.state() == warehouse::RobotState::IDLE, "Robot: initial state = IDLE");
    test(!r.hasPath(), "Robot: initially no path");
}

void test_set_path() {
    warehouse::Robot r(0, {0,0}, {4,0}, 32, 4.0f);
    std::vector<std::pair<int,int>> path = {{0,0},{1,0},{2,0},{3,0},{4,0}};
    r.setPath(path);
    test(r.state() == warehouse::RobotState::MOVING, "Robot: state=MOVING after setPath");
    test(r.hasPath(), "Robot: hasPath() after setPath");
    test(r.pathIndex() == 1, "Robot: pathIndex starts at 1");
}

void test_pause_resume() {
    warehouse::Robot r(0, {0,0}, {9,9}, 32, 4.0f);
    std::vector<std::pair<int,int>> path = {{0,0},{1,0},{2,0}};
    r.setPath(path);
    test(r.state() == warehouse::RobotState::MOVING, "Robot: MOVING before pause");
    r.pause();
    test(r.state() == warehouse::RobotState::PAUSED, "Robot: PAUSED after pause");
    r.resume();
    test(r.state() == warehouse::RobotState::MOVING, "Robot: MOVING after resume");
}

void test_reset() {
    warehouse::Robot r(0, {2,3}, {8,8}, 32, 4.0f);
    std::vector<std::pair<int,int>> path = {{2,3},{3,3},{4,3}};
    r.setPath(path);
    r.update(1.0f);
    r.reset();
    test(r.col() == 2, "Robot: reset col = startPos.col");
    test(r.row() == 3, "Robot: reset row = startPos.row");
    test(r.state() == warehouse::RobotState::IDLE, "Robot: reset state = IDLE");
    test(!r.hasPath(), "Robot: no path after reset");
}

void test_movement() {
    // Robot should move toward next cell with sufficient dt
    warehouse::Robot r(0, {0,0}, {5,0}, 32, 4.0f);
    std::vector<std::pair<int,int>> path = {{0,0},{1,0},{2,0},{3,0},{4,0},{5,0}};
    r.setPath(path);
    // At speed 4.0 cells/sec, 0.3s = 1.2 cells
    r.update(0.3f, 1.0f);
    test(r.col() >= 1, "Robot: moved at least one cell in 0.3s");
}

void test_reach_goal() {
    warehouse::Robot r(0, {0,0}, {2,0}, 32, 10.0f);
    std::vector<std::pair<int,int>> path = {{0,0},{1,0},{2,0}};
    r.setPath(path);
    // High speed, advance enough time
    for (int i = 0; i < 30; ++i) r.update(0.1f, 1.0f);
    test(r.state() == warehouse::RobotState::REACHED_GOAL ||
         r.col() == 2, "Robot: reaches goal position");
}

void test_single_cell_path() {
    warehouse::Robot r(0, {3,3}, {3,3}, 32, 4.0f);
    r.setPath({{3,3}});
    test(r.state() == warehouse::RobotState::REACHED_GOAL,
         "Robot: single-cell path at goal sets REACHED_GOAL");
}

void test_set_goal() {
    warehouse::Robot r(0, {0,0}, {5,5}, 32, 4.0f);
    r.setGoalPos({8,8});
    test(r.goalPos() == std::make_pair(8,8), "Robot: setGoalPos works");
    test(r.state() == warehouse::RobotState::IDLE, "Robot: IDLE after setGoalPos");
}

void test_set_start() {
    warehouse::Robot r(0, {0,0}, {9,9}, 32, 4.0f);
    r.setStartPos({3,4});
    test(r.col() == 3 && r.row() == 4, "Robot: setStartPos moves robot");
    test(r.state() == warehouse::RobotState::IDLE, "Robot: IDLE after setStartPos");
}

void test_astar_robot_integration() {
    warehouse::Grid g(15, 10);
    warehouse::AStarPlanner pf(g);
    warehouse::Robot r(0, {0,0}, {14,9}, 32, 4.0f);

    auto result = pf.findPath({0,0}, {14,9});
    test(result.success, "Robot-AStar: path found");
    r.setPath(result.path);
    test(r.state() == warehouse::RobotState::MOVING, "Robot-AStar: MOVING after setPath");

    // Simulate movement
    float elapsed = 0.0f;
    while (r.state() == warehouse::RobotState::MOVING && elapsed < 30.0f) {
        r.update(0.016f, 2.0f);
        elapsed += 0.016f;
    }
    test(r.state() == warehouse::RobotState::REACHED_GOAL,
         "Robot-AStar: robot reaches goal");
}

}  // anonymous namespace

int main() {
    std::cout << "=== Robot Unit Tests ===\n";
    test_initial_state();
    test_set_path();
    test_pause_resume();
    test_reset();
    test_movement();
    test_reach_goal();
    test_single_cell_path();
    test_set_goal();
    test_set_start();
    test_astar_robot_integration();

    std::cout << "\n=== Results: " << passed << " passed, " << failed << " failed ===\n";
    return (failed == 0) ? 0 : 1;
}
