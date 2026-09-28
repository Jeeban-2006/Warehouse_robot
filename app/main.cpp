// main.cpp - Application entry point.
// 
// Architecture:
//   Main (UI) Thread: SDL2 event loop + rendering
//   Simulation Thread: simulation update loop (60 Hz target)
//   Sensor Thread:     writes sensor data to driver periodically
//
// The two background threads share SimulationEngine protected by stateMutex.
//
#include "core/SimulationEngine.hpp"
#include "ui/Renderer.hpp"
#include "ui/Button.hpp"
#include "system/Logger.hpp"
#include "system/SignalHandler.hpp"
#include "system/TimeUtils.hpp"
#include "driver/SensorDevice.hpp"

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <memory>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>

using namespace warehouse;

// ── Window configuration ──────────────────────────────────────────────────────
static constexpr int WINDOW_W    = 1280;
static constexpr int WINDOW_H    = 780;
static constexpr int PANEL_W     = 280;
static constexpr int GRID_MARGIN = 10;
static constexpr int TARGET_FPS  = 60;

// ── Interaction modes ─────────────────────────────────────────────────────────
enum class UIMode { NORMAL, SET_START, SET_GOAL };

// ── Helper: draw text ─────────────────────────────────────────────────────────
static void drawText(SDL_Renderer* r, TTF_Font* font,
                     const std::string& text, Color c, int x, int y)
{
    if (!font || text.empty()) return;
    SDL_Surface* s = TTF_RenderText_Blended(font, text.c_str(), c.sdl());
    if (!s) return;
    SDL_Texture* t = SDL_CreateTextureFromSurface(r, s);
    if (t) {
        SDL_Rect d{x, y, s->w, s->h};
        SDL_RenderCopy(r, t, nullptr, &d);
        SDL_DestroyTexture(t);
    }
    SDL_FreeSurface(s);
}

// ── Panel drawing ─────────────────────────────────────────────────────────────
static void drawPanel(SDL_Renderer* renderer, TTF_Font* fontLg, TTF_Font* fontMd, TTF_Font* fontSm,
                      const SimulationEngine& sim, const std::vector<std::unique_ptr<Button>>& buttons,
                      int px, int ph, UIMode uiMode, bool sensorOnline, float fps,
                      const SensorDevice* sensorDev)
{
    // Panel background
    SDL_SetRenderDrawColor(renderer, Colors::PANEL_BG.r, Colors::PANEL_BG.g, Colors::PANEL_BG.b, 255);
    SDL_Rect pr{px, 0, PANEL_W, ph};
    SDL_RenderFillRect(renderer, &pr);
    SDL_SetRenderDrawColor(renderer, Colors::PANEL_BORDER.r, Colors::PANEL_BORDER.g, Colors::PANEL_BORDER.b, 255);
    SDL_RenderDrawLine(renderer, px, 0, px, ph);

    int ty = 10;

    // Title
    drawText(renderer, fontLg, "WAREHOUSE ROBOT", Colors::TEXT_BRIGHT, px+10, ty); ty += 24;
    drawText(renderer, fontSm, "Pathfinding Simulator v2.0", Colors::TEXT_DIM, px+10, ty); ty += 20;

    // Divider
    SDL_SetRenderDrawColor(renderer, Colors::PANEL_BORDER.r, Colors::PANEL_BORDER.g, Colors::PANEL_BORDER.b, 255);
    SDL_RenderDrawLine(renderer, px+5, ty, px+PANEL_W-5, ty); ty += 8;

    // Interaction mode indicator
    if (uiMode != UIMode::NORMAL) {
        std::string modeStr = (uiMode == UIMode::SET_START) ? "CLICK: Set Start" : "CLICK: Set Goal";
        SDL_SetRenderDrawColor(renderer, Colors::ACCENT.r, Colors::ACCENT.g, Colors::ACCENT.b, 60);
        SDL_Rect mb{px+5, ty-2, PANEL_W-10, 18}; SDL_RenderFillRect(renderer, &mb);
        drawText(renderer, fontSm, modeStr, Colors::ACCENT, px+10, ty); ty += 20;
    }

    // Buttons
    for (const auto& btn : buttons)
        btn->draw(renderer, fontMd);

    // Stats section
    ty = 590;
    SDL_SetRenderDrawColor(renderer, Colors::PANEL_BORDER.r, Colors::PANEL_BORDER.g, Colors::PANEL_BORDER.b, 255);
    SDL_RenderDrawLine(renderer, px+5, ty-4, px+PANEL_W-5, ty-4);

    const auto& met = sim.metrics();
    const auto& robot = sim.robot();

    auto stat = [&](const std::string& k, const std::string& v) {
        drawText(renderer, fontSm, k, Colors::TEXT_DIM, px+8, ty);
        drawText(renderer, fontSm, v, Colors::TEXT, px+140, ty);
        ty += 16;
    };

    stat("State:",    simStateStr(sim.state()));
    stat("Robot pos:", "(" + std::to_string(robot.col()) + "," + std::to_string(robot.row()) + ")");
    stat("Robot:",    robotStateStr(robot.state()));
    stat("Path:",     std::to_string(met.lastPathLength) + " cells");
    stat("Explored:", std::to_string(met.lastNodesExplored));
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << met.lastPlanTimeMs;
    stat("Plan time:", oss.str() + " ms");
    stat("Replans:",  std::to_string(met.replanCount));
    std::ostringstream t2;
    t2 << std::fixed << std::setprecision(1) << met.elapsedSeconds;
    stat("Elapsed:",  t2.str() + "s");
    stat("FPS:",      std::to_string(static_cast<int>(fps)));

    // Sensor status
    ty += 4;
    SDL_SetRenderDrawColor(renderer, Colors::PANEL_BORDER.r, Colors::PANEL_BORDER.g, Colors::PANEL_BORDER.b, 255);
    SDL_RenderDrawLine(renderer, px+5, ty-2, px+PANEL_W-5, ty-2); ty += 2;
    Color sensorCol = sensorOnline ? Colors::SUCCESS : Colors::ERROR;
    std::string sensorStr = sensorOnline ? "ONLINE" : "OFFLINE (sim mode)";
    drawText(renderer, fontSm, "Sensor:", Colors::TEXT_DIM, px+8, ty);
    drawText(renderer, fontSm, sensorStr, sensorCol, px+80, ty); ty += 16;

    // Status message
    ty = ph - 45;
    SDL_SetRenderDrawColor(renderer, Colors::PANEL_BORDER.r, Colors::PANEL_BORDER.g, Colors::PANEL_BORDER.b, 255);
    SDL_RenderDrawLine(renderer, px+5, ty-4, px+PANEL_W-5, ty-4);
    drawText(renderer, fontSm, sim.statusMessage(), Colors::ACCENT, px+5, ty); ty += 18;

    // Help strip
    drawText(renderer, fontSm, "SPACE=Start/Pause  R=Reset", Colors::TEXT_DIM, px+5, ty); ty += 14;
    drawText(renderer, fontSm, "F=Path  D=Debug  S=SetStart  G=SetGoal", Colors::TEXT_DIM, px+5, ty);
}

// ── Simulation thread ─────────────────────────────────────────────────────────
static void simulationThread(SimulationEngine* sim, std::atomic<bool>& running)
{
    LOG_INFO("Simulation thread started");
    using Clk = std::chrono::steady_clock;
    constexpr auto TARGET_DT = std::chrono::duration<double>(1.0 / 120.0);

    auto prev = Clk::now();
    while (running) {
        auto now = Clk::now();
        double dt = std::chrono::duration<double>(now - prev).count();
        prev = now;

        {
            std::lock_guard<std::mutex> lock(sim->stateMutex);
            sim->update(static_cast<float>(std::min(dt, 0.05)));
        }

        auto elapsed = Clk::now() - now;
        if (elapsed < TARGET_DT)
            std::this_thread::sleep_for(TARGET_DT - elapsed);
    }
    LOG_INFO("Simulation thread stopped");
}

// ── Sensor thread ─────────────────────────────────────────────────────────────
static void sensorThread(SimulationEngine* sim, SensorDevice* sensor,
                          std::atomic<bool>& running)
{
    LOG_INFO("Sensor thread started");
    int seq = 0;
    while (running) {
        // Read sensor data from simulation and write to driver
        SimulationEngine::SensorReading sr;
        {
            std::lock_guard<std::mutex> lock(sim->stateMutex);
            sr = sim->computeSensorReading();
        }

        if (sensor && sensor->isOpen()) {
            SensorReading devSr;
            devSr.frontDistance    = sr.front;
            devSr.rearDistance     = sr.rear;
            devSr.leftDistance     = sr.left;
            devSr.rightDistance    = sr.right;
            devSr.obstacleDetected = sr.obstacleDetected;
            devSr.sequence         = ++seq;
            devSr.robotCol         = sim->robot().col();
            devSr.robotRow         = sim->robot().row();
            devSr.robotState       = static_cast<int>(sim->robot().state());
            sensor->writeSensor(devSr);
        }

        // 4 Hz update rate for sensor
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    LOG_INFO("Sensor thread stopped");
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    (void)argc; (void)argv;

    // ── Logger ────────────────────────────────────────────────────────────
    Logger::instance().init("logs/warehouse_robot.log", LogLevel::INFO);
    LOG_INFO("=== Autonomous Warehouse Robot Simulator ===");

    // ── Signal handling ───────────────────────────────────────────────────
    SignalHandler::install();

    // ── SDL2 init ─────────────────────────────────────────────────────────
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        LOG_FATAL(std::string("SDL_Init failed: ") + SDL_GetError());
        return 1;
    }
    if (TTF_Init() != 0) {
        LOG_FATAL(std::string("TTF_Init failed: ") + TTF_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "Autonomous Warehouse Robot Pathfinding Simulator",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_W, WINDOW_H, SDL_WINDOW_SHOWN);
    if (!window) {
        LOG_FATAL(std::string("SDL_CreateWindow failed: ") + SDL_GetError());
        TTF_Quit(); SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) {
        LOG_FATAL(std::string("SDL_CreateRenderer failed: ") + SDL_GetError());
        SDL_DestroyWindow(window); TTF_Quit(); SDL_Quit();
        return 1;
    }
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    LOG_INFO("SDL2 window created: " + std::to_string(WINDOW_W) + "x" + std::to_string(WINDOW_H));

    // ── Fonts ─────────────────────────────────────────────────────────────
    // Try system fonts; fall back to default
    TTF_Font* fontLg = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 15);
    if (!fontLg) fontLg = TTF_OpenFont("/usr/share/fonts/TTF/DejaVuSans-Bold.ttf", 15);
    if (!fontLg) { LOG_WARN("fontLg not found, UI text may be invisible."); }

    TTF_Font* fontMd = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 13);
    if (!fontMd) fontMd = TTF_OpenFont("/usr/share/fonts/TTF/DejaVuSans.ttf", 13);
    if (!fontMd) { LOG_WARN("fontMd not found."); }

    TTF_Font* fontSm = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 11);
    if (!fontSm) fontSm = fontMd;

    // ── Layout ───────────────────────────────────────────────────────────
    int panelX   = WINDOW_W - PANEL_W;
    int gridAreaW = panelX;
    int gridAreaH = WINDOW_H;

    SimConfig cfg;
    cfg.gridCols    = 30;
    cfg.gridRows    = 22;
    int cellW = (gridAreaW - 2*GRID_MARGIN) / cfg.gridCols;
    int cellH = (gridAreaH - 2*GRID_MARGIN) / cfg.gridRows;
    cfg.cellSize    = std::max(8, std::min(cellW, cellH));

    SDL_Rect gridRect{GRID_MARGIN, GRID_MARGIN,
                      cfg.gridCols * cfg.cellSize,
                      cfg.gridRows * cfg.cellSize};

    LOG_INFO("Grid: " + std::to_string(cfg.gridCols) + "x" + std::to_string(cfg.gridRows) +
             " cell=" + std::to_string(cfg.cellSize));

    // ── Simulation ────────────────────────────────────────────────────────
    SimulationEngine sim(cfg);
    LOG_INFO("Simulation engine initialized");

    // ── Sensor device ─────────────────────────────────────────────────────
    auto sensor = std::make_unique<SensorDevice>();
    bool sensorOnline = sensor->open();
    if (sensorOnline)
        LOG_INFO("Sensor device online: " + std::string(SensorDevice::DEFAULT_DEVICE));
    else
        LOG_WARN("Sensor device offline: " + sensor->lastError() + " (simulation mode)");

    // ── Renderer ─────────────────────────────────────────────────────────
    Renderer warehouseRenderer(renderer, gridRect, cfg.cellSize, fontLg, fontMd, fontSm);

    // ── State ─────────────────────────────────────────────────────────────
    UIMode uiMode = UIMode::NORMAL;
    bool densityCycle = false;
    float densityValues[] = {0.10f, 0.20f, 0.32f};
    const char* densityLabels[] = {"Low", "Medium", "High"};
    int densityIdx = 1;
    float speedValues[] = {0.5f, 1.0f, 2.0f, 3.0f};
    int speedIdx = 1;
    sim.speedMultiplier = speedValues[speedIdx];

    // ── Buttons (in panel) ────────────────────────────────────────────────
    std::vector<std::unique_ptr<Button>> buttons;
    int bx = panelX + 10, bw = PANEL_W - 20, bh = 28;
    int by = 58;

    auto addBtn = [&](const std::string& label, std::function<void()> fn,
                      Color col = Colors::BTN_NORMAL) {
        buttons.push_back(std::make_unique<Button>(bx, by, bw, bh, label, fn, col));
        by += bh + 4;
        return buttons.back().get();
    };

    Button* btnStart  = addBtn("START",  [&]{
        std::lock_guard<std::mutex> lock(sim.stateMutex);
        if (sim.state() == SimState::RUNNING) sim.pause();
        else if (sim.state() == SimState::PAUSED) sim.resume();
        else sim.start();
    }, {50,160,90});

    Button* btnReset  = addBtn("RESET",  [&]{
        std::lock_guard<std::mutex> lock(sim.stateMutex);
        sim.reset();
        uiMode = UIMode::NORMAL;
    }, {160,80,50});

    addBtn("FIND PATH", [&]{
        std::lock_guard<std::mutex> lock(sim.stateMutex);
        sim.findPathOnly();
    });

    by += 4;
    addBtn("Default Map", [&]{
        std::lock_guard<std::mutex> lock(sim.stateMutex);
        sim.loadDefaultMap();
    });

    addBtn("Random Map", [&]{
        std::lock_guard<std::mutex> lock(sim.stateMutex);
        sim.generateRandomMap(densityValues[densityIdx]);
        sim.reset();
    });

    addBtn("Clear Map", [&]{
        std::lock_guard<std::mutex> lock(sim.stateMutex);
        sim.clearObstacles();
    });

    addBtn(std::string("Density: ") + densityLabels[densityIdx], [&]{
        densityIdx = (densityIdx + 1) % 3;
        buttons[6]->setLabel(std::string("Density: ") + densityLabels[densityIdx]);
    });

    by += 4;
    addBtn(std::string("Speed: ") + std::to_string(static_cast<int>(speedValues[speedIdx])) + "x",
           [&]{
        speedIdx = (speedIdx + 1) % 4;
        sim.speedMultiplier = speedValues[speedIdx];
        std::ostringstream os;
        os << std::fixed << std::setprecision(1) << speedValues[speedIdx];
        buttons[7]->setLabel("Speed: " + os.str() + "x");
    });

    by += 4;
    addBtn("Set Start", [&]{
        uiMode = (uiMode == UIMode::SET_START) ? UIMode::NORMAL : UIMode::SET_START;
    });

    addBtn("Set Goal", [&]{
        uiMode = (uiMode == UIMode::SET_GOAL) ? UIMode::NORMAL : UIMode::SET_GOAL;
    });

    by += 4;
    addBtn("Path: ON",  [&]{
        sim.showPath = !sim.showPath;
        buttons[10]->setLabel(sim.showPath ? "Path: ON" : "Path: OFF");
        buttons[10]->setToggled(sim.showPath);
    });
    buttons.back()->setToggled(true);

    addBtn("Explored: OFF", [&]{
        sim.showExplored = !sim.showExplored;
        buttons[11]->setLabel(sim.showExplored ? "Explored: ON" : "Explored: OFF");
        buttons[11]->setToggled(sim.showExplored);
    });

    addBtn("Dyn Obs: ON", [&]{
        sim.dynamicObsEnabled = !sim.dynamicObsEnabled;
        buttons[12]->setLabel(sim.dynamicObsEnabled ? "Dyn Obs: ON" : "Dyn Obs: OFF");
        buttons[12]->setToggled(sim.dynamicObsEnabled);
    });
    buttons.back()->setToggled(true);

    addBtn("Debug: OFF", [&]{
        sim.debugMode = !sim.debugMode;
        buttons[13]->setLabel(sim.debugMode ? "Debug: ON" : "Debug: OFF");
        buttons[13]->setToggled(sim.debugMode);
    });

    LOG_INFO("UI initialized: " + std::to_string(buttons.size()) + " buttons");

    // ── Threads ───────────────────────────────────────────────────────────
    std::atomic<bool> threadsRunning{true};

    std::thread simThread(simulationThread, &sim, std::ref(threadsRunning));
    std::thread senThread(sensorThread, &sim, sensor.get(), std::ref(threadsRunning));

    LOG_INFO("Application ready. Enter simulation loop.");

    // ── Main loop ─────────────────────────────────────────────────────────
    using Clk = std::chrono::steady_clock;
    auto frameStart  = Clk::now();
    auto frameTarget = std::chrono::duration<double>(1.0 / TARGET_FPS);
    float pulseT = 0.0f;
    float fps    = 60.0f;
    int   frameCount = 0;
    auto  fpsTimer = Clk::now();

    while (!SignalHandler::shutdownRequested()) {
        auto now = Clk::now();
        float dt = static_cast<float>(
            std::chrono::duration<double>(now - frameStart).count());
        frameStart = now;
        pulseT += dt;
        ++frameCount;

        // FPS calculation
        auto fpsDur = std::chrono::duration<double>(now - fpsTimer).count();
        if (fpsDur >= 1.0) {
            fps = static_cast<float>(frameCount) / static_cast<float>(fpsDur);
            frameCount = 0;
            fpsTimer   = now;
        }

        // ── Update button labels based on sim state ────────────────────
        {
            auto st = sim.state();
            if (st == SimState::RUNNING)
                btnStart->setLabel("PAUSE");
            else if (st == SimState::PAUSED)
                btnStart->setLabel("RESUME");
            else
                btnStart->setLabel("START");
        }

        // ── Events ────────────────────────────────────────────────────
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                SignalHandler::shutdownRequested().store(true);
                break;
            }

            // Pass event to all buttons
            for (auto& btn : buttons)
                btn->handleEvent(event);

            // Keyboard
            if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                case SDLK_ESCAPE:
                    uiMode = UIMode::NORMAL;
                    break;
                case SDLK_SPACE: {
                    std::lock_guard<std::mutex> lock(sim.stateMutex);
                    auto st = sim.state();
                    if (st == SimState::RUNNING) sim.pause();
                    else if (st == SimState::PAUSED) sim.resume();
                    else sim.start();
                    break;
                }
                case SDLK_r: {
                    std::lock_guard<std::mutex> lock(sim.stateMutex);
                    sim.reset();
                    uiMode = UIMode::NORMAL;
                    break;
                }
                case SDLK_f: {
                    std::lock_guard<std::mutex> lock(sim.stateMutex);
                    sim.findPathOnly();
                    break;
                }
                case SDLK_s:
                    uiMode = (uiMode == UIMode::SET_START) ? UIMode::NORMAL : UIMode::SET_START;
                    break;
                case SDLK_g:
                    uiMode = (uiMode == UIMode::SET_GOAL) ? UIMode::NORMAL : UIMode::SET_GOAL;
                    break;
                case SDLK_d:
                    sim.debugMode = !sim.debugMode;
                    break;
                }
            }

            // Mouse on grid
            if (event.type == SDL_MOUSEBUTTONDOWN) {
                int mx = event.button.x, my = event.button.y;
                // Check if in grid area
                if (mx >= gridRect.x && mx < gridRect.x + gridRect.w &&
                    my >= gridRect.y && my < gridRect.y + gridRect.h)
                {
                    int col = (mx - gridRect.x) / cfg.cellSize;
                    int row = (my - gridRect.y) / cfg.cellSize;
                    std::lock_guard<std::mutex> lock(sim.stateMutex);
                    if (uiMode == UIMode::SET_START) {
                        sim.setRobotStart(col, row);
                        uiMode = UIMode::NORMAL;
                    } else if (uiMode == UIMode::SET_GOAL) {
                        sim.setRobotGoal(col, row);
                        uiMode = UIMode::NORMAL;
                    } else if (event.button.button == SDL_BUTTON_LEFT) {
                        sim.setObstacle(col, row, true);
                    } else if (event.button.button == SDL_BUTTON_RIGHT) {
                        sim.removeObstacle(col, row);
                    }
                }
            }
            if (event.type == SDL_MOUSEMOTION && (event.motion.state & SDL_BUTTON_LMASK)) {
                int mx = event.motion.x, my = event.motion.y;
                if (mx >= gridRect.x && mx < gridRect.x + gridRect.w &&
                    my >= gridRect.y && my < gridRect.y + gridRect.h &&
                    uiMode == UIMode::NORMAL)
                {
                    int col = (mx - gridRect.x) / cfg.cellSize;
                    int row = (my - gridRect.y) / cfg.cellSize;
                    std::lock_guard<std::mutex> lock(sim.stateMutex);
                    sim.setObstacle(col, row, true);
                }
            }
        }

        // ── Render ────────────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lock(sim.stateMutex);
            warehouseRenderer.render(sim, pulseT, dt);
        }
        drawPanel(renderer, fontLg, fontMd, fontSm, sim, buttons,
                  panelX, WINDOW_H, uiMode, sensorOnline, fps, sensor.get());

        SDL_RenderPresent(renderer);

        // Frame rate cap
        auto elapsed = Clk::now() - frameStart;
        if (elapsed < frameTarget)
            std::this_thread::sleep_for(frameTarget - elapsed);
    }

    // ── Shutdown ─────────────────────────────────────────────────────────
    LOG_INFO("Shutdown requested. Stopping threads...");
    threadsRunning = false;
    simThread.join();
    senThread.join();

    sensor->stopMonitor();
    sensor->close();

    if (fontSm && fontSm != fontMd) TTF_CloseFont(fontSm);
    if (fontMd) TTF_CloseFont(fontMd);
    if (fontLg) TTF_CloseFont(fontLg);

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();

    LOG_INFO("Application exited cleanly.");
    Logger::instance().flush();
    return 0;
}
