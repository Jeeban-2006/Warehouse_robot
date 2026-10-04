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

    // Check death
    if (m_robot->state() == RobotState::ERROR) {
        m_statusMessage = "CRITICAL: BATTERY 0%. SYSTEM DEAD.";
        m_state = SimState::PAUSED;
        return;
    }

    // Smart Battery Management System (BMS)
    bool needsEmergencyCharge = false;
    std::pair<int,int> nearestCharger = findNearestCharger();

    if (m_robot->state() != RobotState::CHARGING && m_robot->state() != RobotState::IDLE) {
        float battery = m_robot->batteryPercentage();
        // 1. Hard minimum limit
        if (battery < 15.0f) {
            needsEmergencyCharge = true;
        } 
        // 2. Predictive BMS based on A* distance
        else if (m_robot->hasPath()) {
            // Distance to complete current goal
            float distToGoal = (float)(m_robot->path().size() - m_robot->pathIndex());
            // Distance from goal to the nearest charger
            float distFromGoalToCharger = std::abs(m_robot->goalPos().first - nearestCharger.first) + 
                                          std::abs(m_robot->goalPos().second - nearestCharger.second);
            
            // Movement uses ~0.15% battery per grid cell
            float batteryNeeded = (distToGoal + distFromGoalToCharger) * 0.15f;
            
            // If we don't have enough to finish task AND reach charger (plus 5% safety buffer)
            if (battery < batteryNeeded + 5.0f) {
                needsEmergencyCharge = true;
            }
        }

        if (needsEmergencyCharge && m_robot->goalPos() != nearestCharger) {
            setRobotGoal(nearestCharger.first, nearestCharger.second);
            triggerReplan();
            m_statusMessage = "BMS PREDICTION: Low battery for mission. Routing to nearest charger!";
        }
    }

    if (m_pauseTimer > 0.0f) {
        m_pauseTimer -= (dt * speedMultiplier);
        return; // Pause the robot logic, but dynamic obstacles and sensors still run
    }

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
            m_statusMessage = "Path blocked by obstacle! Replanning...";
        }
    } else if (!m_robot->hasPath() && m_robot->state() == RobotState::IDLE) {
        // Deadlock fix: If robot has no path but should be doing a task or charging
        if (m_robot->needsCharging() || m_robot->currentTask() != nullptr) {
            if (m_replanTimer >= m_config.replanInterval) {
                m_replanTimer = 0.0f;
                triggerReplan();
                m_statusMessage = "Retrying path to goal...";
            }
        }
    }

    // State machine logic
    if (m_robot->state() == RobotState::REACHED_GOAL) {
        auto pos = m_robot->pos();
        auto task = m_robot->currentTask();

        if (m_grid->getCell(pos.first, pos.second).type == CellType::CHARGING_STATION) {
            m_robot->setState(RobotState::CHARGING);
            m_statusMessage = "Charging... (Please Wait)";
            m_pauseTimer = 3.0f; // 3 seconds to fully charge visually
        }
        else if (task) {
            if (task->status == TaskStatus::ASSIGNED && pos == task->pickupLocation) {
                task->status = TaskStatus::PICKING;
                m_robot->setState(RobotState::PICKING);
                m_statusMessage = "PICKING UP ITEM... (Please Wait)";
                m_pauseTimer = 2.0f;
            }
            else if (task->status == TaskStatus::PICKING && pos == task->deliveryLocation) {
                task->status = TaskStatus::DELIVERING;
                m_robot->setState(RobotState::DELIVERING);
                m_statusMessage = "DELIVERING ITEM... (Please Wait)";
                m_pauseTimer = 2.0f;
            }
            else {
                // Wrong goal reached (likely due to replan failure to old goal)
                if (task->status == TaskStatus::ASSIGNED) setRobotGoal(task->pickupLocation.first, task->pickupLocation.second);
                else setRobotGoal(task->deliveryLocation.first, task->deliveryLocation.second);
            }
        } else {
            m_state = SimState::COMPLETED;
            m_statusMessage = "Goal reached!";
            m_robot->setState(RobotState::IDLE);
        }
    }

    if (m_robot->state() == RobotState::CHARGING && m_pauseTimer <= 0.0f) {
        m_robot->chargeBattery(100.0f); // fully charge
        m_statusMessage = "Fully Charged! Resuming task...";
        // Resume task
        auto task = m_robot->currentTask();
        if (task) {
            if (task->status == TaskStatus::ASSIGNED) setRobotGoal(task->pickupLocation.first, task->pickupLocation.second);
            else if (task->status == TaskStatus::PICKING) setRobotGoal(task->deliveryLocation.first, task->deliveryLocation.second);
            triggerReplan();
        } else {
            m_robot->setState(RobotState::IDLE);
        }
    }
    else if (m_robot->state() == RobotState::PICKING && m_pauseTimer <= 0.0f) {
        auto task = m_robot->currentTask();
        if (task) {
            setRobotGoal(task->deliveryLocation.first, task->deliveryLocation.second);
            triggerReplan();
            m_statusMessage = "Picked up item. Routing to delivery.";
        }
    }
    else if (m_robot->state() == RobotState::DELIVERING && m_pauseTimer <= 0.0f) {
        auto task = m_robot->currentTask();
        if (task) {
            task->status = TaskStatus::COMPLETED;
            m_statusMessage = "Task Completed!";
        }
        m_robot->clearTask();
        m_robot->setState(RobotState::IDLE);
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
    if (m_grid->inBounds(c, r)) {
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

void SimulationEngine::loadDefaultMap(int layoutType) {
    m_grid->generateRealisticWarehouse(layoutType);
    
    // Add dynamic obstacles mimicking patrol workers
    m_dynObs.clear();
    m_dynObs.push_back(std::make_unique<DynamicObstacle>(0, 10, 6, ObstacleAxis::HORIZONTAL, 10, 50, 2.0f, m_config.cellSize));
    m_dynObs.push_back(std::make_unique<DynamicObstacle>(1, 30, 2, ObstacleAxis::VERTICAL, 2, 40, 3.0f, m_config.cellSize));
    m_dynObs.push_back(std::make_unique<DynamicObstacle>(2, 20, 11, ObstacleAxis::VERTICAL, 5, 40, 2.5f, m_config.cellSize));
    m_dynObs.push_back(std::make_unique<DynamicObstacle>(3, 10, 16, ObstacleAxis::HORIZONTAL, 10, 45, 1.8f, m_config.cellSize));
    m_dynObs.push_back(std::make_unique<DynamicObstacle>(4, 38, 5, ObstacleAxis::VERTICAL, 5, 35, 3.2f, m_config.cellSize));
    m_dynObs.push_back(std::make_unique<DynamicObstacle>(5, 10, 31, ObstacleAxis::HORIZONTAL, 10, 50, 2.2f, m_config.cellSize));
    m_dynObs.push_back(std::make_unique<DynamicObstacle>(6, 48, 5, ObstacleAxis::VERTICAL, 5, 40, 2.8f, m_config.cellSize));
    m_dynObs.push_back(std::make_unique<DynamicObstacle>(7, 5, 21, ObstacleAxis::HORIZONTAL, 5, 25, 2.0f, m_config.cellSize));

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

std::pair<int,int> SimulationEngine::findNearestCharger() const {
    std::pair<int,int> best = {-1, -1};
    float minDist = 999999.0f;
    for (int r = 0; r < m_grid->rows(); ++r) {
        for (int c = 0; c < m_grid->cols(); ++c) {
            if (m_grid->getCell(c, r).type == CellType::CHARGING_STATION) {
                float dist = std::abs(m_robot->col() - c) + std::abs(m_robot->row() - r);
                if (dist < minDist) {
                    minDist = dist;
                    best = {c, r};
                }
            }
        }
    }
    if (best.first == -1) return {1, 5};
    return best;
}

} // namespace warehouse
