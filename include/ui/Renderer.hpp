// Renderer.hpp - SDL2-based warehouse renderer.
#pragma once

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <string>
#include <memory>

namespace warehouse {

class SimulationEngine;

struct Color {
    Uint8 r, g, b, a{255};
    SDL_Color sdl() const { return {r, g, b, a}; }
};

// ── Design tokens ────────────────────────────────────────────────────────────
namespace Colors {
    inline constexpr Color BG           {18,  20,  30};
    inline constexpr Color PANEL_BG     {24,  26,  40};
    inline constexpr Color PANEL_BORDER {60,  65,  90};
    inline constexpr Color CELL_FREE    {30,  33,  50};
    inline constexpr Color CELL_GRID    {45,  48,  68};
    inline constexpr Color OBSTACLE     {70,  75,  95};
    inline constexpr Color OBSTACLE_TOP {90,  95, 115};
    inline constexpr Color PATH         {80, 200, 255};
    inline constexpr Color PATH_EXP     {40,  60,  90};
    inline constexpr Color ROBOT        {50, 200, 130};
    inline constexpr Color ROBOT_OUT    {30, 150,  90};
    inline constexpr Color GOAL         {255,200,  40};
    inline constexpr Color GOAL_OUT     {200,150,  20};
    inline constexpr Color DYN_OBS      {220, 80,  80};
    inline constexpr Color DYN_OBS_OUT  {180, 50,  50};
    inline constexpr Color START        {60, 220, 180};
    inline constexpr Color TEXT         {210,215, 235};
    inline constexpr Color TEXT_DIM     {120,125, 150};
    inline constexpr Color TEXT_BRIGHT  {255,255, 255};
    inline constexpr Color ACCENT       {80, 160, 255};
    inline constexpr Color SUCCESS      {60, 200, 100};
    inline constexpr Color WARNING      {255,180,  40};
    inline constexpr Color ERROR        {220, 60,  60};
    inline constexpr Color BTN_NORMAL   {50,  55,  80};
    inline constexpr Color BTN_HOVER    {70,  80, 115};
    inline constexpr Color BTN_ACTIVE   {80, 155, 245};
    inline constexpr Color BTN_DISABLED {38,  40,  60};
}

class Renderer {
public:
    Renderer(SDL_Renderer* r, SDL_Rect gridRect, int cellSize,
             TTF_Font* fontLg, TTF_Font* fontMd, TTF_Font* fontSm);

    void render(const SimulationEngine& sim, float pulseT, float dt);

private:
    SDL_Renderer* m_renderer;
    SDL_Rect      m_gridRect;
    int           m_cellSize;
    TTF_Font*     m_fontLg;
    TTF_Font*     m_fontMd;
    TTF_Font*     m_fontSm;

    void setColor(Color c);
    void drawFilledRect(int x, int y, int w, int h);
    void drawRect(int x, int y, int w, int h);
    void drawCircle(int cx, int cy, int r);
    void drawFilledCircle(int cx, int cy, int r);
    void drawLine(int x1, int y1, int x2, int y2);
    void drawText(const std::string& text, TTF_Font* font, Color c, int x, int y);
    void drawTextCentered(const std::string& text, TTF_Font* font, Color c, int cx, int y);

    void drawGridBackground(const SimulationEngine& sim);
    void drawExploredNodes(const SimulationEngine& sim);
    void drawPath(const SimulationEngine& sim);
    void drawStaticObstacles(const SimulationEngine& sim);
    void drawGoal(const SimulationEngine& sim, float pulseT);
    void drawStart(const SimulationEngine& sim);
    void drawDynamicObstacles(const SimulationEngine& sim);
    void drawRobot(const SimulationEngine& sim);
    void drawCompletedOverlay(const SimulationEngine& sim);
    void drawNoPathOverlay(const SimulationEngine& sim);
    void drawDebugOverlay(const SimulationEngine& sim, float fps);
    void drawReplanBanner(const SimulationEngine& sim);
};

}  // namespace warehouse
