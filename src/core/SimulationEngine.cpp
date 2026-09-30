#include "core/SimulationEngine.hpp"
#include <iostream>

namespace warehouse {

std::string simStateStr(SimState s) {
    switch(s) {
        case SimState::IDLE: return "IDLE";
        case SimState::RUNNING: return "RUNNING";
        case SimState::PAUSED: return "PAUSED";
        case SimState::NO_PATH: return "NO_PATH";
        case SimState::COMPLETED: return "COMPLETED";
        default: return "UNKNOWN";
    }
}

SimulationEngine::SimulationEngine(const SimConfig& config)
    : m_config(config), m_state(SimState::IDLE), m_sensorProc(15.0f) 
{
    m_grid = std::make_unique<Grid>(config.gridCols, config.gridRows);
    m_robot = std::make_unique<Robot>(1, std::make_pair(1,1), std::make_pair(config.gridCols-2, config.gridRows-2), config.cellSize, config.robotSpeed);
    m_planner = std::make_unique<AStarPlanner>(*m_grid);
    m_statusMessage = "Engine initialized.";
    loadDefaultMap();
}

void SimulationEngine::start() {
    if (m_state == SimState::IDLE || m_state == SimState::NO_PATH || m_state == SimState::COMPLETED) {
        m_state = SimState::RUNNING;
        m_metrics.elapsedSeconds = 0.0f;
        triggerReplan();
    }
}

void SimulationEngine::pause() {
    if (m_state == SimState::RUNNING) {
        m_state = SimState::PAUSED;
        m_robot->pause();
        m_statusMessage = "Simulation paused.";
    }
}

void SimulationEngine::resume() {
    if (m_state == SimState::PAUSED) {
        m_state = SimState::RUNNING;
        m_robot->resume();
        m_statusMessage = "Simulation resumed.";
    }
}

void SimulationEngine::reset() {
    m_state = SimState::IDLE;
    m_robot->reset();
    m_dynObs.clear();
    // In a real system, we'd reload the default dynamic obstacles, but for now we'll just clear them or reload them via loadDefaultMap.
    loadDefaultMap(); // Note: this will also reset the grid and robot positions.
    // Actually, let's just clear and let the user generate them or reload map explicitly.
    m_metrics = Metrics();
    m_replanTimer = 0.0f;
    m_statusMessage = "Simulation reset.";
}

void SimulationEngine::update(float dt) {
    if (m_state != SimState::RUNNING) return;
    
    m_metrics.elapsedSeconds += dt;
    m_replanTimer += dt;

    // Update dynamic obstacles
    if (dynamicObsEnabled) {
        for (auto& o : m_dynObs) o->update(dt * speedMultiplier);
    }

    // LiDAR & Sensors
    updateSensorData();

    // Check battery state
    if (m_robot->needsCharging() && m_robot->state() != RobotState::CHARGING && m_robot->state() != RobotState::IDLE) {
        // Simple logic to find a charging station (just one specific pos for demo)
        setRobotGoal(1, 5); // Usually where chargers are placed in realistic map
        triggerReplan();
        m_statusMessage = "LOW BATTERY! Routing to charger.";
    }

    // Robot state execution
    m_robot->update(dt, speedMultiplier);

    // Replanning based on interval or path blockage
    if (m_robot->hasPath() && m_replanTimer >= m_config.replanInterval) {
        m_replanTimer = 0.0f;
        PosSet extra;
        if (dynamicObsEnabled) {
            for (auto& o : m_dynObs) {
                auto p = o->pos();
                extra.insert(p);
            }
        }
        if (!m_planner->isPathValid(m_robot->path(), extra)) {
            m_metrics.replanCount++;
            triggerReplan();
        }
    }

    if (m_robot->state() == RobotState::REACHED_GOAL && m_robot->currentTask() == nullptr) {
        m_state = SimState::COMPLETED;
        m_statusMessage = "Goal reached!";
    }
}

void SimulationEngine::triggerReplan() {
    PosSet extra;
    if (dynamicObsEnabled) {
        for (auto& o : m_dynObs) {
            auto p = o->pos();
            extra.insert(p);
        }
    }

    auto start = m_robot->pos();
    auto goal = m_robot->goalPos();
    auto res = m_planner->findPath(start, goal, extra);

    m_metrics.totalPathCalls++;
    m_metrics.lastPlanTimeMs = res.planningTimeMs;
    m_metrics.lastNodesExplored = res.nodesExplored;

    if (res.success) {
        m_metrics.lastPathLength = res.pathLength;
        m_robot->setPath(res.path);
        m_robot->setState(RobotState::MOVING);
        m_statusMessage = "Path found (" + std::to_string(res.pathLength) + " cells)";
        if (m_state == SimState::NO_PATH) m_state = SimState::RUNNING;
    } else {
        m_robot->clearPath();
        m_robot->setState(RobotState::BLOCKED);
        m_state = SimState::NO_PATH;
        m_statusMessage = "No valid path: " + res.message;
    }
}

void SimulationEngine::findPathOnly() {
    triggerReplan();
}

void SimulationEngine::setRobotStart(int c, int r) {
    if (m_grid->inBounds(c, r) && m_grid->isFree(c, r)) {
        m_robot->setStartPos({c, r});
        m_statusMessage = "Start position updated.";
    }
}

void SimulationEngine::setRobotGoal(int c, int r) {
    if (m_grid->inBounds(c, r) && m_grid->isFree(c, r)) {
        m_robot->setGoalPos({c, r});
        m_statusMessage = "Goal position updated.";
        if (m_state == SimState::RUNNING) {
            triggerReplan();
        }
    }
}

void SimulationEngine::assignTask(int pickupCol, int pickupRow, int dropCol, int dropRow) {
    auto task = std::make_shared<Task>(++m_taskCounter, std::make_pair(pickupCol, pickupRow), std::make_pair(dropCol, dropRow), 1, "Package");
    m_robot->assignTask(task);
    setRobotGoal(pickupCol, pickupRow); // First go to pickup
    m_statusMessage = "Task Assigned! Routing to pickup.";
    start();
}

void SimulationEngine::setObstacle(int c, int r, bool isObs) {
    if (!m_grid->inBounds(c, r)) return;
    auto rp = m_robot->pos();
    if (c == rp.first && r == rp.second) return;
    auto gp = m_robot->goalPos();
    if (c == gp.first && r == gp.second) return;
    
    m_grid->setObstacle(c, r, isObs);
}

void SimulationEngine::removeObstacle(int c, int r) {
    m_grid->setObstacle(c, r, false);
}

void SimulationEngine::clearObstacles() {
    m_grid->clear();
    m_dynObs.clear();
    m_statusMessage = "Map cleared.";
}

void SimulationEngine::loadDefaultMap() {
    m_grid->generateRealisticWarehouse();
    
    // Add dynamic obstacles mimicking patrol workers
    m_dynObs.clear();
    m_dynObs.push_back(std::make_unique<DynamicObstacle>(0, 10, 6, ObstacleAxis::HORIZONTAL, 10, 50, 2.0f, m_config.cellSize));
    m_dynObs.push_back(std::make_unique<DynamicObstacle>(1, 30, 2, ObstacleAxis::VERTICAL, 2, 40, 3.0f, m_config.cellSize));

    m_robot->setStartPos({1, 1});
    m_robot->setGoalPos({m_config.gridCols-5, m_config.gridRows-3});
    m_statusMessage = "Realistic warehouse loaded.";
}

void SimulationEngine::generateRandomMap(float density) {
    std::vector<std::pair<int,int>> protectedCells = {m_robot->pos(), m_robot->goalPos()};
    m_grid->generateRandom(density, protectedCells);
    m_dynObs.clear();
    m_statusMessage = "Random map generated.";
}

void SimulationEngine::updateSensorData() {
    float rx = m_robot->col() + 0.5f;
    float ry = m_robot->row() + 0.5f;
    m_lastScan = m_sensorProc.perform360Scan(*m_grid, rx, ry, 36);
}

SimulationEngine::SensorReading SimulationEngine::computeSensorReading() {
    SensorReading sr;
    m_sensorProc.getDirectionalDistances(*m_grid, m_robot->col(), m_robot->row(), sr.front, sr.rear, sr.left, sr.right);
    sr.seq = ++m_sensorSeq;
    
    sr.obstacleDetected = false;
    if ((sr.front > 0 && sr.front <= 3) ||
        (sr.rear > 0 && sr.rear <= 3) ||
        (sr.left > 0 && sr.left <= 3) ||
        (sr.right > 0 && sr.right <= 3)) {
        sr.obstacleDetected = true;
    }
    
    // Check dynamic obstacles for interference
    for (const auto& o : m_dynObs) {
        int dx = std::abs(o->pos().first - m_robot->col());
        int dy = std::abs(o->pos().second - m_robot->row());
        if (dx + dy <= 3) sr.obstacleDetected = true;
    }
    return sr;
}

} // namespace warehouse
