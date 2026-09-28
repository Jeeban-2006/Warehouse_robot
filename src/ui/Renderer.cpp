// Renderer.cpp - SDL2 warehouse rendering implementation.
#include "ui/Renderer.hpp"
#include "core/SimulationEngine.hpp"
#include "core/Robot.hpp"

#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>

namespace warehouse {

Renderer::Renderer(SDL_Renderer* r, SDL_Rect gridRect, int cellSize,
                   TTF_Font* fontLg, TTF_Font* fontMd, TTF_Font* fontSm)
    : m_renderer(r), m_gridRect(gridRect), m_cellSize(cellSize),
      m_fontLg(fontLg), m_fontMd(fontMd), m_fontSm(fontSm)
{}

// ── Primitive helpers ─────────────────────────────────────────────────────────
void Renderer::setColor(Color c) {
    SDL_SetRenderDrawColor(m_renderer, c.r, c.g, c.b, c.a);
}

void Renderer::drawFilledRect(int x, int y, int w, int h) {
    SDL_Rect r{x, y, w, h};
    SDL_RenderFillRect(m_renderer, &r);
}

void Renderer::drawRect(int x, int y, int w, int h) {
    SDL_Rect r{x, y, w, h};
    SDL_RenderDrawRect(m_renderer, &r);
}

void Renderer::drawLine(int x1, int y1, int x2, int y2) {
    SDL_RenderDrawLine(m_renderer, x1, y1, x2, y2);
}

void Renderer::drawCircle(int cx, int cy, int r) {
    // Midpoint circle algorithm
    int x = r, y = 0, err = 0;
    while (x >= y) {
        SDL_RenderDrawPoint(m_renderer, cx+x, cy+y);
        SDL_RenderDrawPoint(m_renderer, cx+y, cy+x);
        SDL_RenderDrawPoint(m_renderer, cx-y, cy+x);
        SDL_RenderDrawPoint(m_renderer, cx-x, cy+y);
        SDL_RenderDrawPoint(m_renderer, cx-x, cy-y);
        SDL_RenderDrawPoint(m_renderer, cx-y, cy-x);
        SDL_RenderDrawPoint(m_renderer, cx+y, cy-x);
        SDL_RenderDrawPoint(m_renderer, cx+x, cy-y);
        if (err <= 0) { ++y; err += 2*y+1; }
        else          { --x; err -= 2*x+1; }
    }
}

void Renderer::drawFilledCircle(int cx, int cy, int r) {
    for (int dy = -r; dy <= r; ++dy) {
        int dx = static_cast<int>(std::sqrt(static_cast<double>(r*r - dy*dy)));
        SDL_RenderDrawLine(m_renderer, cx-dx, cy+dy, cx+dx, cy+dy);
    }
}

void Renderer::drawText(const std::string& text, TTF_Font* font, Color c, int x, int y) {
    if (!font || text.empty()) return;
    SDL_Surface* surf = TTF_RenderText_Blended(font, text.c_str(), c.sdl());
    if (!surf) return;
    SDL_Texture* tex = SDL_CreateTextureFromSurface(m_renderer, surf);
    if (tex) {
        SDL_Rect dst{x, y, surf->w, surf->h};
        SDL_RenderCopy(m_renderer, tex, nullptr, &dst);
        SDL_DestroyTexture(tex);
    }
    SDL_FreeSurface(surf);
}

void Renderer::drawTextCentered(const std::string& text, TTF_Font* font, Color c, int cx, int y) {
    if (!font || text.empty()) return;
    int w = 0, h = 0;
    TTF_SizeText(font, text.c_str(), &w, &h);
    drawText(text, font, c, cx - w/2, y);
}

// ── Top-level render ──────────────────────────────────────────────────────────
void Renderer::render(const SimulationEngine& sim, float pulseT, float dt) {
    (void)dt;
    // Clear with background color
    setColor(Colors::BG);
    SDL_RenderClear(m_renderer);

    drawGridBackground(sim);
    if (sim.showExplored) drawExploredNodes(sim);
    if (sim.showPath)     drawPath(sim);
    drawStaticObstacles(sim);
    drawGoal(sim, pulseT);
    drawStart(sim);
    drawDynamicObstacles(sim);
    drawRobot(sim);
    drawReplanBanner(sim);

    auto st = sim.state();
    if (st == SimState::COMPLETED) drawCompletedOverlay(sim);
    if (st == SimState::NO_PATH)   drawNoPathOverlay(sim);
    if (sim.debugMode)             drawDebugOverlay(sim, 0.0f);
}

// ── Grid background ───────────────────────────────────────────────────────────
void Renderer::drawGridBackground(const SimulationEngine& sim) {
    const auto& grid = sim.grid();
    int ox = m_gridRect.x, oy = m_gridRect.y;
    int cs = m_cellSize;
    for (int r = 0; r < grid.rows(); ++r) {
        for (int c = 0; c < grid.cols(); ++c) {
            int rx = ox + c*cs, ry = oy + r*cs;
            setColor(Colors::CELL_FREE);
            drawFilledRect(rx, ry, cs, cs);
            setColor(Colors::CELL_GRID);
            drawRect(rx, ry, cs, cs);
        }
    }
}

// ── Explored nodes ────────────────────────────────────────────────────────────
void Renderer::drawExploredNodes(const SimulationEngine& sim) {
    const auto& result = sim.lastPathResult();
    int ox = m_gridRect.x, oy = m_gridRect.y;
    int cs = m_cellSize;
    SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_BLEND);
    setColor({Colors::PATH_EXP.r, Colors::PATH_EXP.g, Colors::PATH_EXP.b, 80});
    for (const auto& [c, r] : result.explored) {
        drawFilledRect(ox+c*cs, oy+r*cs, cs, cs);
    }
    SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_NONE);
}

// ── Path ─────────────────────────────────────────────────────────────────────
void Renderer::drawPath(const SimulationEngine& sim) {
    const auto& robot = sim.robot();
    if (!robot.hasPath()) return;
    const auto& path = robot.path();
    int ox = m_gridRect.x, oy = m_gridRect.y;
    int cs = m_cellSize;
    int n = static_cast<int>(path.size());

    // Draw path dots
    for (int i = 1; i < n; ++i) {
        auto [c, r] = path[static_cast<std::size_t>(i)];
        float t = (n > 1) ? 0.4f + 0.6f * (float(i) / float(n-1)) : 1.0f;
        Color col{
            static_cast<Uint8>(Colors::PATH.r * t),
            static_cast<Uint8>(Colors::PATH.g * t),
            static_cast<Uint8>(Colors::PATH.b * t)
        };
        setColor(col);
        int px = ox + c*cs + cs/4, py = oy + r*cs + cs/4;
        drawFilledRect(px, py, cs/2, cs/2);
    }

    // Draw line connecting path cells
    if (n > 1) {
        setColor({Colors::PATH.r, Colors::PATH.g, Colors::PATH.b, 160});
        for (int i = 0; i < n-1; ++i) {
            auto [c1,r1] = path[static_cast<std::size_t>(i)];
            auto [c2,r2] = path[static_cast<std::size_t>(i+1)];
            drawLine(ox+c1*cs+cs/2, oy+r1*cs+cs/2,
                     ox+c2*cs+cs/2, oy+r2*cs+cs/2);
        }
    }
}

// ── Static obstacles ──────────────────────────────────────────────────────────
void Renderer::drawStaticObstacles(const SimulationEngine& sim) {
    const auto& grid = sim.grid();
    int ox = m_gridRect.x, oy = m_gridRect.y;
    int cs = m_cellSize;
    for (int r = 0; r < grid.rows(); ++r) {
        for (int c = 0; c < grid.cols(); ++c) {
            if (!grid.isObstacle(c, r)) continue;
            int rx = ox+c*cs, ry = oy+r*cs;
            setColor(Colors::OBSTACLE);
            drawFilledRect(rx, ry, cs, cs);
            setColor(Colors::OBSTACLE_TOP);
            drawLine(rx, ry, rx+cs-1, ry);
            drawLine(rx, ry, rx, ry+cs-1);
            setColor({50,55,75});
            drawLine(rx, ry+cs-1, rx+cs-1, ry+cs-1);
            drawLine(rx+cs-1, ry, rx+cs-1, ry+cs-1);
        }
    }
}

// ── Goal ─────────────────────────────────────────────────────────────────────
void Renderer::drawGoal(const SimulationEngine& sim, float pulseT) {
    auto [gc, gr] = sim.robot().goalPos();
    int ox = m_gridRect.x, oy = m_gridRect.y;
    int cs = m_cellSize;
    int cx = ox + gc*cs + cs/2;
    int cy = oy + gr*cs + cs/2;
    int r  = cs/2 - 3;

    // Pulse ring
    float pulse = std::abs(std::sin(pulseT * 2.5f));
    int   pr    = r + static_cast<int>(pulse * 5);
    SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_BLEND);
    setColor({Colors::GOAL.r, Colors::GOAL.g, Colors::GOAL.b,
              static_cast<Uint8>(60 + pulse * 80)});
    drawFilledCircle(cx, cy, pr);
    SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_NONE);

    setColor(Colors::GOAL);
    drawFilledCircle(cx, cy, r);
    setColor(Colors::GOAL_OUT);
    drawCircle(cx, cy, r);

    drawTextCentered("G", m_fontSm, {30,20,0}, cx, cy - 7);
}

// ── Start marker ──────────────────────────────────────────────────────────────
void Renderer::drawStart(const SimulationEngine& sim) {
    auto [sc, sr] = sim.robot().startPos();
    if (std::make_pair(sc,sr) == sim.robot().pos()) return;
    int ox = m_gridRect.x, oy = m_gridRect.y;
    int cs = m_cellSize;
    int cx = ox + sc*cs + cs/2, cy = oy + sr*cs + cs/2;
    int r  = cs/2 - 4;
    setColor(Colors::START);
    drawFilledCircle(cx, cy, r);
    setColor({30,160,120});
    drawCircle(cx, cy, r);
    drawTextCentered("S", m_fontSm, {0,40,30}, cx, cy-7);
}

// ── Dynamic obstacles ─────────────────────────────────────────────────────────
void Renderer::drawDynamicObstacles(const SimulationEngine& sim) {
    int ox = m_gridRect.x, oy = m_gridRect.y;
    int cs = m_cellSize;
    for (const auto& obs : sim.dynObs()) {
        int cx = ox + static_cast<int>(obs->pixelX());
        int cy = oy + static_cast<int>(obs->pixelY());
        int r  = cs/2 - 2;
        setColor(Colors::DYN_OBS);
        drawFilledCircle(cx, cy, r);
        setColor(Colors::DYN_OBS_OUT);
        drawCircle(cx, cy, r);
        drawTextCentered("D", m_fontSm, {255,220,220}, cx, cy-7);
    }
}

// ── Robot ─────────────────────────────────────────────────────────────────────
void Renderer::drawRobot(const SimulationEngine& sim) {
    const auto& robot = sim.robot();
    int ox = m_gridRect.x, oy = m_gridRect.y;
    int cs = m_cellSize;
    int cx = ox + static_cast<int>(robot.pixelX());
    int cy = oy + static_cast<int>(robot.pixelY());
    int r  = cs/2 - 3;

    // Body color based on state
    Color bodyCol = Colors::ROBOT;
    switch(robot.state()) {
        case RobotState::PAUSED:       bodyCol = {100,180,140}; break;
        case RobotState::ERROR:        bodyCol = {200, 80, 80}; break;
        case RobotState::BLOCKED:      bodyCol = {200,150, 50}; break;
        case RobotState::REACHED_GOAL: bodyCol = {100,220,160}; break;
        default: break;
    }
    setColor(bodyCol);
    drawFilledCircle(cx, cy, r);
    setColor(Colors::ROBOT_OUT);
    drawCircle(cx, cy, r);

    // Eyes
    auto [dc, dr] = robot.facing();
    int eo = r/3;
    int ex = cx + dc*eo, ey = cy + dr*eo;
    setColor({240,240,240});
    drawFilledCircle(ex + (-dr)*3, ey + dc*3, 2);
    drawFilledCircle(ex + dr*3,    ey + (-dc)*3, 2);

    // State dot
    Color dotCol = Colors::TEXT_DIM;
    switch(robot.state()) {
        case RobotState::MOVING:       dotCol = Colors::SUCCESS; break;
        case RobotState::PAUSED:       dotCol = Colors::WARNING; break;
        case RobotState::PLANNING:     dotCol = Colors::ACCENT;  break;
        case RobotState::REACHED_GOAL: dotCol = Colors::SUCCESS; break;
        case RobotState::ERROR:        dotCol = Colors::ERROR;   break;
        default: break;
    }
    setColor(dotCol);
    drawFilledCircle(cx+r-4, cy-r+4, 4);
}

// ── Overlays ──────────────────────────────────────────────────────────────────
void Renderer::drawCompletedOverlay(const SimulationEngine& sim) {
    // Semi-transparent overlay
    SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_BLEND);
    setColor({0,0,0,60});
    SDL_RenderFillRect(m_renderer, &m_gridRect);
    SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_NONE);

    const auto& met = sim.metrics();
    int cw = 340, ch = 200;
    int cx = m_gridRect.x + m_gridRect.w/2;
    int cy = m_gridRect.y + m_gridRect.h/2;
    int rx = cx - cw/2, ry = cy - ch/2;

    setColor({20,30,25});
    drawFilledRect(rx, ry, cw, ch);
    setColor(Colors::SUCCESS);
    drawRect(rx, ry, cw, ch);

    int ty = ry + 10;
    drawTextCentered("SIMULATION COMPLETE", m_fontMd, Colors::SUCCESS, cx, ty); ty += 30;
    drawTextCentered("Path: " + std::to_string(met.lastPathLength) + " cells",
                     m_fontSm, Colors::TEXT, cx, ty); ty += 22;
    drawTextCentered("Nodes explored: " + std::to_string(met.lastNodesExplored),
                     m_fontSm, Colors::TEXT, cx, ty); ty += 22;
    drawTextCentered("Replans: " + std::to_string(met.replanCount),
                     m_fontSm, Colors::WARNING, cx, ty); ty += 22;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << met.elapsedSeconds;
    drawTextCentered("Time: " + oss.str() + "s",
                     m_fontSm, Colors::TEXT, cx, ty); ty += 22;
    drawTextCentered("Press R to restart", m_fontSm, Colors::TEXT_DIM, cx, ty);
}

void Renderer::drawNoPathOverlay(const SimulationEngine& sim) {
    int cx = m_gridRect.x + m_gridRect.w/2;
    int ty = m_gridRect.y + 20;

    SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_BLEND);
    setColor({40,20,20,200});
    drawFilledRect(cx-210, ty-8, 420, 55);
    SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_NONE);
    setColor(Colors::ERROR);
    drawRect(cx-210, ty-8, 420, 55);

    drawTextCentered("NO VALID PATH FOUND", m_fontMd, Colors::ERROR, cx, ty);
    drawTextCentered("Remove obstacles then press F or SPACE",
                     m_fontSm, Colors::TEXT_DIM, cx, ty+25);
    (void)sim;
}

void Renderer::drawReplanBanner(const SimulationEngine& sim) {
    if (!sim.replanningBanner()) return;
    int cx = m_gridRect.x + m_gridRect.w/2;
    int ty = m_gridRect.y + m_gridRect.h - 50;

    SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_BLEND);
    setColor({200,60,60,200});
    drawFilledRect(cx-150, ty-6, 300, 30);
    SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_NONE);
    drawTextCentered("REPLANNING PATH...", m_fontMd, Colors::TEXT_BRIGHT, cx, ty);
}

void Renderer::drawDebugOverlay(const SimulationEngine& sim, float fps) {
    const auto& robot = sim.robot();
    const auto& met   = sim.metrics();
    int sx = m_gridRect.x + 4;
    int sy = m_gridRect.y + 4;

    auto line = [&](const std::string& s) {
        SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_BLEND);
        int tw = 0, th = 0;
        if (m_fontSm) TTF_SizeText(m_fontSm, s.c_str(), &tw, &th);
        setColor({0,0,0,140});
        drawFilledRect(sx-2, sy, tw+6, th);
        SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_NONE);
        drawText(s, m_fontSm, {200,255,200}, sx, sy);
        sy += 15;
    };

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    line("FPS: " + (fps > 0 ? (oss.str(), std::to_string(static_cast<int>(fps))) : "?"));
    line("State: " + std::string(simStateStr(sim.state())));
    line("Robot: (" + std::to_string(robot.col()) + "," + std::to_string(robot.row()) + ")");
    line("RobotState: " + std::string(robotStateStr(robot.state())));
    line("PathIdx: " + std::to_string(robot.pathIndex()) + "/" +
         std::to_string(robot.path().size()));
    line("PathCalls: " + std::to_string(met.totalPathCalls));
    line("Replans: " + std::to_string(met.replanCount));
    line("PlanMs: " + [&]{
        std::ostringstream o; o << std::fixed << std::setprecision(2) << met.lastPlanTimeMs;
        return o.str();
    }());
    line("Speed: " + std::to_string(static_cast<int>(sim.speedMultiplier * 10)/10) + "x");
}

}  // namespace warehouse
