#pragma once
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include "core/SimulationEngine.hpp"

namespace warehouse {

struct Color {
    Uint8 r, g, b, a{255};
    SDL_Color sdl() const { return {r, g, b, a}; }
};

namespace Colors {
    const Color BG           = {18, 20, 30, 255};
    const Color PANEL_BG     = {24, 26, 40, 255};
    const Color PANEL_BORDER = {40, 45, 65, 255};

    const Color CELL_FREE    = {30, 33, 50, 255};
    const Color CELL_OBSTACLE= {70, 75, 95, 255};
    
    // Warehouse Objects
    const Color SHELF           = {210, 105, 30, 255};   // Chocolate/Orange
    const Color CHARGING_STATION= {0, 191, 255, 255};    // Cyan
    const Color LOADING_ZONE    = {255, 215, 0, 255};    // Gold
    const Color PICKUP_STATION  = {147, 112, 219, 255};  // Purple

    const Color PATH         = {80, 200, 255, 120};
    const Color PATH_LINE    = {80, 200, 255, 200};
    const Color EXPLORED     = {60, 70, 100, 100};

    const Color ROBOT        = {50, 200, 130, 255};
    const Color ROBOT_ERROR  = {220, 50, 50, 255};
    const Color ROBOT_CHARGE = {50, 150, 255, 255};
    const Color GOAL         = {255, 200, 40, 255};
    const Color DYN_OBS      = {220, 80, 80, 255};
    
    const Color LIDAR_RAY    = {255, 50, 50, 100};

    const Color TEXT         = {220, 225, 240, 255};
    const Color TEXT_DIM     = {130, 140, 160, 255};
    const Color TEXT_BRIGHT  = {255, 255, 255, 255};

    const Color BTN_NORMAL   = {45, 50, 75, 255};
    const Color BTN_HOVER    = {60, 65, 90, 255};
    const Color BTN_ACTIVE   = {80, 120, 200, 255};
    const Color BTN_DISABLED = {30, 33, 45, 255};

    const Color ACCENT       = {80, 160, 255, 255};
    const Color SUCCESS      = {60, 200, 100, 255};
    const Color WARNING      = {255, 180, 40, 255};
    const Color ERROR        = {220, 60, 60, 255};
}

class Renderer {
public:
    Renderer(SDL_Renderer* renderer, SDL_Rect gridRect, int cellSize,
             TTF_Font* fontLg, TTF_Font* fontMd, TTF_Font* fontSm);
    
    void render(const SimulationEngine& sim, float pulseTime, float dt);

private:
    void drawGrid(const Grid& grid);
    void drawPath(const std::vector<std::pair<int,int>>& path, size_t currentIndex);
    void drawRobot(const Robot& robot, float pulseT);
    void drawDynamicObstacles(const std::vector<std::unique_ptr<DynamicObstacle>>& dynObs);
    void drawLidar(const LidarScan& scan, float rx, float ry);

    SDL_Renderer* m_ren;
    SDL_Rect m_gridRect;
    int m_cellSize;
    TTF_Font* m_fontLg;
    TTF_Font* m_fontMd;
    TTF_Font* m_fontSm;
};

} // namespace warehouse
