#include "ui/Renderer.hpp"
#include <cmath>

namespace warehouse {

Renderer::Renderer(SDL_Renderer* renderer, SDL_Rect gridRect, int cellSize,
                   TTF_Font* fontLg, TTF_Font* fontMd, TTF_Font* fontSm)
    : m_ren(renderer), m_gridRect(gridRect), m_cellSize(cellSize),
      m_fontLg(fontLg), m_fontMd(fontMd), m_fontSm(fontSm) {}

void Renderer::render(const SimulationEngine& sim, float pulseT, float dt) {
    // Clear screen
    SDL_SetRenderDrawColor(m_ren, Colors::BG.r, Colors::BG.g, Colors::BG.b, 255);
    SDL_RenderClear(m_ren);

    // Draw grid background
    SDL_SetRenderDrawColor(m_ren, 20, 22, 32, 255);
    SDL_RenderFillRect(m_ren, &m_gridRect);

    drawGrid(sim.grid());

    // Draw explored nodes if enabled
    if (sim.showExplored) {
        // Explored nodes not currently exposed in sim natively without tracking it
        // A* planner could expose it, but skipped for brevity in rendering
    }

    if (sim.showPath && sim.robot().hasPath()) {
        drawPath(sim.robot().path(), sim.robot().pathIndex());
    }

    // Draw goal position
    auto goal = sim.robot().goalPos();
    SDL_Rect gRect{m_gridRect.x + goal.first * m_cellSize + 2,
                   m_gridRect.y + goal.second * m_cellSize + 2,
                   m_cellSize - 4, m_cellSize - 4};
    SDL_SetRenderDrawColor(m_ren, Colors::GOAL.r, Colors::GOAL.g, Colors::GOAL.b, 255);
    SDL_RenderFillRect(m_ren, &gRect);

    if (sim.dynamicObsEnabled) {
        drawDynamicObstacles(sim.dynObs());
    }

    drawRobot(sim.robot(), pulseT);

    if (sim.debugMode) {
        drawLidar(sim.latestLidarScan(), sim.robot().centreX(), sim.robot().centreY());
    }
}

void Renderer::drawGrid(const Grid& grid) {
    for (int r = 0; r < grid.rows(); ++r) {
        for (int c = 0; c < grid.cols(); ++c) {
            SDL_Rect cellRect{m_gridRect.x + c * m_cellSize, m_gridRect.y + r * m_cellSize, m_cellSize, m_cellSize};
            
            auto type = grid.getCell(c, r).type;
            Color col = Colors::CELL_FREE;
            
            switch (type) {
                case CellType::OBSTACLE: col = Colors::CELL_OBSTACLE; break;
                case CellType::SHELF: col = Colors::SHELF; break;
                case CellType::CHARGING_STATION: col = Colors::CHARGING_STATION; break;
                case CellType::LOADING_ZONE: col = Colors::LOADING_ZONE; break;
                case CellType::PICKUP_STATION: col = Colors::PICKUP_STATION; break;
                default: col = Colors::CELL_FREE; break;
            }

            SDL_SetRenderDrawColor(m_ren, col.r, col.g, col.b, col.a);
            SDL_RenderFillRect(m_ren, &cellRect);

            // Draw grid lines
            SDL_SetRenderDrawColor(m_ren, 35, 40, 55, 255);
            SDL_RenderDrawRect(m_ren, &cellRect);
        }
    }
}

void Renderer::drawPath(const std::vector<std::pair<int,int>>& path, size_t currentIndex) {
    if (path.size() < 2) return;
    SDL_SetRenderDrawColor(m_ren, Colors::PATH_LINE.r, Colors::PATH_LINE.g, Colors::PATH_LINE.b, Colors::PATH_LINE.a);
    
    for (size_t i = currentIndex; i < path.size() - 1; ++i) {
        int x1 = m_gridRect.x + path[i].first * m_cellSize + m_cellSize / 2;
        int y1 = m_gridRect.y + path[i].second * m_cellSize + m_cellSize / 2;
        int x2 = m_gridRect.x + path[i+1].first * m_cellSize + m_cellSize / 2;
        int y2 = m_gridRect.y + path[i+1].second * m_cellSize + m_cellSize / 2;
        SDL_RenderDrawLine(m_ren, x1, y1, x2, y2);
    }
}

void Renderer::drawRobot(const Robot& robot, float pulseT) {
    int rx = m_gridRect.x + static_cast<int>(robot.centreX()) - m_cellSize / 2;
    int ry = m_gridRect.y + static_cast<int>(robot.centreY()) - m_cellSize / 2;

    SDL_Rect rRect{rx + 2, ry + 2, m_cellSize - 4, m_cellSize - 4};
    
    Color rCol = Colors::ROBOT;
    if (robot.state() == RobotState::ERROR || robot.state() == RobotState::BLOCKED) rCol = Colors::ROBOT_ERROR;
    if (robot.state() == RobotState::CHARGING) rCol = Colors::ROBOT_CHARGE;
    if (robot.state() == RobotState::PICKING || robot.state() == RobotState::DELIVERING) {
        rCol.r = 255; rCol.g = 150; rCol.b = 0; // Orange for task execution
    }

    SDL_SetRenderDrawColor(m_ren, rCol.r, rCol.g, rCol.b, 255);
    SDL_RenderFillRect(m_ren, &rRect);

    // Pulse effect
    if (robot.state() == RobotState::MOVING) {
        int pSize = static_cast<int>((std::sin(pulseT * 10.0f) * 0.5f + 0.5f) * 4.0f);
        SDL_Rect pulseRect{rx - pSize, ry - pSize, m_cellSize + pSize * 2, m_cellSize + pSize * 2};
        SDL_SetRenderDrawColor(m_ren, rCol.r, rCol.g, rCol.b, 100);
        SDL_RenderDrawRect(m_ren, &pulseRect);
    }
    
    // Battery indicator
    int batH = static_cast<int>((robot.batteryPercentage() / 100.0f) * (m_cellSize - 4));
    SDL_Rect batRect{rx + m_cellSize - 6, ry + 2 + ((m_cellSize-4)-batH), 4, batH};
    if (robot.batteryPercentage() > 50) SDL_SetRenderDrawColor(m_ren, 0, 255, 0, 255);
    else if (robot.batteryPercentage() > 20) SDL_SetRenderDrawColor(m_ren, 255, 255, 0, 255);
    else SDL_SetRenderDrawColor(m_ren, 255, 0, 0, 255);
    SDL_RenderFillRect(m_ren, &batRect);
}

void Renderer::drawDynamicObstacles(const std::vector<std::unique_ptr<DynamicObstacle>>& dynObs) {
    for (const auto& o : dynObs) {
        int ox = m_gridRect.x + static_cast<int>(o->pixelX()) - m_cellSize / 2;
        int oy = m_gridRect.y + static_cast<int>(o->pixelY()) - m_cellSize / 2;
        SDL_Rect oRect{ox + 2, oy + 2, m_cellSize - 4, m_cellSize - 4};
        SDL_SetRenderDrawColor(m_ren, Colors::DYN_OBS.r, Colors::DYN_OBS.g, Colors::DYN_OBS.b, 255);
        SDL_RenderFillRect(m_ren, &oRect);
    }
}

void Renderer::drawLidar(const LidarScan& scan, float rx, float ry) {
    int startX = m_gridRect.x + static_cast<int>(rx);
    int startY = m_gridRect.y + static_cast<int>(ry);
    
    SDL_SetRenderDrawColor(m_ren, Colors::LIDAR_RAY.r, Colors::LIDAR_RAY.g, Colors::LIDAR_RAY.b, Colors::LIDAR_RAY.a);
    
    for (const auto& ray : scan.rays) {
        float rad = ray.angleDeg * (M_PI / 180.0f);
        int endX = startX + static_cast<int>(std::cos(rad) * ray.distance * m_cellSize);
        int endY = startY + static_cast<int>(std::sin(rad) * ray.distance * m_cellSize);
        SDL_RenderDrawLine(m_ren, startX, startY, endX, endY);
        
        if (ray.hitObstacle) {
            SDL_Rect hitRect{endX - 2, endY - 2, 4, 4};
            SDL_SetRenderDrawColor(m_ren, 255, 255, 0, 255);
            SDL_RenderFillRect(m_ren, &hitRect);
            SDL_SetRenderDrawColor(m_ren, Colors::LIDAR_RAY.r, Colors::LIDAR_RAY.g, Colors::LIDAR_RAY.b, Colors::LIDAR_RAY.a);
        }
    }
}

} // namespace warehouse
