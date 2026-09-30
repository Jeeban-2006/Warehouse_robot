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
#include <chrono>
#include <memory>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>

using namespace warehouse;

static constexpr int WINDOW_W    = 1280;
static constexpr int WINDOW_H    = 760;
static constexpr int PANEL_W     = 280;
static constexpr int GRID_MARGIN = 10;
static constexpr int TARGET_FPS  = 60;

enum class UIMode { NORMAL, SET_START, SET_GOAL, ASSIGN_TASK };

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

static void drawPanel(SDL_Renderer* renderer, TTF_Font* fontLg, TTF_Font* fontMd, TTF_Font* fontSm,
                      const SimulationEngine& sim, const std::vector<std::unique_ptr<Button>>& buttons,
                      int px, int ph, UIMode uiMode, bool sensorOnline, float fps,
                      const SensorDevice* sensorDev)
{
    SDL_SetRenderDrawColor(renderer, Colors::PANEL_BG.r, Colors::PANEL_BG.g, Colors::PANEL_BG.b, 255);
    SDL_Rect pr{px, 0, PANEL_W, ph};
    SDL_RenderFillRect(renderer, &pr);
    SDL_SetRenderDrawColor(renderer, Colors::PANEL_BORDER.r, Colors::PANEL_BORDER.g, Colors::PANEL_BORDER.b, 255);
    SDL_RenderDrawLine(renderer, px, 0, px, ph);

    int ty = 10;
    drawText(renderer, fontLg, "WAREHOUSE ROBOT", Colors::TEXT_BRIGHT, px+10, ty); ty += 24;
    drawText(renderer, fontSm, "Autonomous Task Engine v3.0", Colors::TEXT_DIM, px+10, ty); ty += 20;

    SDL_SetRenderDrawColor(renderer, Colors::PANEL_BORDER.r, Colors::PANEL_BORDER.g, Colors::PANEL_BORDER.b, 255);
    SDL_RenderDrawLine(renderer, px+5, ty, px+PANEL_W-5, ty); ty += 8;

    if (uiMode != UIMode::NORMAL) {
        std::string modeStr = "CLICK: Set Start";
        if (uiMode == UIMode::SET_GOAL) modeStr = "CLICK: Set Goal";
        else if (uiMode == UIMode::ASSIGN_TASK) modeStr = "CLICK: Assign Pickup";
        SDL_SetRenderDrawColor(renderer, Colors::ACCENT.r, Colors::ACCENT.g, Colors::ACCENT.b, 60);
        SDL_Rect mb{px+5, ty-2, PANEL_W-10, 18}; SDL_RenderFillRect(renderer, &mb);
        drawText(renderer, fontSm, modeStr, Colors::ACCENT, px+10, ty); ty += 20;
    }

    // Buttons
    for (const auto& btn : buttons) btn->draw(renderer, fontMd);

    // Dynamic stats
    ty = 460;
    SDL_SetRenderDrawColor(renderer, Colors::PANEL_BORDER.r, Colors::PANEL_BORDER.g, Colors::PANEL_BORDER.b, 255);
    SDL_RenderDrawLine(renderer, px+5, ty-4, px+PANEL_W-5, ty-4);

    const auto& met = sim.metrics();
    const auto& robot = sim.robot();

    auto stat = [&](const std::string& k, const std::string& v, Color c = Colors::TEXT) {
        drawText(renderer, fontSm, k, Colors::TEXT_DIM, px+8, ty);
        drawText(renderer, fontSm, v, c, px+100, ty);
        ty += 16;
    };

    drawText(renderer, fontMd, "ROBOT STATUS", Colors::TEXT_BRIGHT, px+8, ty); ty += 18;
    stat("State:", robotStateStr(robot.state()));
    std::ostringstream bat;
    bat << std::fixed << std::setprecision(1) << robot.batteryPercentage() << "%";
    Color batCol = robot.batteryPercentage() > 20.0f ? Colors::SUCCESS : Colors::ERROR;
    stat("Battery:", bat.str(), batCol);
    stat("Pos:", "(" + std::to_string(robot.col()) + "," + std::to_string(robot.row()) + ")");
    ty += 8;

    drawText(renderer, fontMd, "MISSION", Colors::TEXT_BRIGHT, px+8, ty); ty += 18;
    if (robot.currentTask()) {
        stat("Task ID:", std::to_string(robot.currentTask()->id));
        stat("Pickup:", "(" + std::to_string(robot.currentTask()->pickupLocation.first) + "," + std::to_string(robot.currentTask()->pickupLocation.second) + ")");
        stat("Drop:", "(" + std::to_string(robot.currentTask()->deliveryLocation.first) + "," + std::to_string(robot.currentTask()->deliveryLocation.second) + ")");
    } else {
        stat("Task:", "None");
    }
    ty += 8;

    drawText(renderer, fontMd, "NAVIGATION", Colors::TEXT_BRIGHT, px+8, ty); ty += 18;
    std::ostringstream pt; pt << std::fixed << std::setprecision(2) << met.lastPlanTimeMs;
    stat("A* Time:", pt.str() + " ms");
    stat("Replans:", std::to_string(met.replanCount));
    ty += 8;

    // Sensor Status
    SDL_SetRenderDrawColor(renderer, Colors::PANEL_BORDER.r, Colors::PANEL_BORDER.g, Colors::PANEL_BORDER.b, 255);
    SDL_RenderDrawLine(renderer, px+5, ty-4, px+PANEL_W-5, ty-4);
    
    drawText(renderer, fontMd, "SYSTEM", Colors::TEXT_BRIGHT, px+8, ty); ty += 18;
    Color sensorCol = sensorOnline ? Colors::SUCCESS : Colors::ERROR;
    std::string sensorStr = sensorOnline ? "CONNECTED" : "SIMULATED";
    stat("Driver:", sensorStr, sensorCol);
    stat("FPS:", std::to_string(static_cast<int>(fps)));

    // Status message
    ty = ph - 40;
    SDL_SetRenderDrawColor(renderer, Colors::PANEL_BORDER.r, Colors::PANEL_BORDER.g, Colors::PANEL_BORDER.b, 255);
    SDL_RenderDrawLine(renderer, px+5, ty-4, px+PANEL_W-5, ty-4);
    drawText(renderer, fontSm, sim.statusMessage(), Colors::ACCENT, px+5, ty); ty += 18;
}

static void simulationThread(SimulationEngine* sim, std::atomic<bool>& running) {
    LOG_INFO("Simulation thread started");
    using Clk = std::chrono::steady_clock;
    constexpr auto TARGET_DT = std::chrono::duration<double>(1.0 / 60.0);
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
        if (elapsed < TARGET_DT) std::this_thread::sleep_for(TARGET_DT - elapsed);
    }
}

static void sensorThread(SimulationEngine* sim, SensorDevice* sensor, std::atomic<bool>& running) {
    LOG_INFO("Sensor thread started");
    int seq = 0;
    while (running) {
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
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
}

int main(int argc, char* argv[]) {
    (void)argc; (void)argv;
    Logger::instance().init("logs/warehouse_robot.log", LogLevel::INFO);
    SignalHandler::install();

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0 || TTF_Init() != 0) return 1;

    SDL_Window* window = SDL_CreateWindow("Warehouse Robot Simulator v3.0",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_W, WINDOW_H, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    TTF_Font* fontLg = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 15);
    if (!fontLg) fontLg = TTF_OpenFont("/usr/share/fonts/TTF/DejaVuSans-Bold.ttf", 15);
    TTF_Font* fontMd = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 12);
    if (!fontMd) fontMd = fontLg;
    TTF_Font* fontSm = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 11);
    if (!fontSm) fontSm = fontMd;

    int panelX = WINDOW_W - PANEL_W;
    SimConfig cfg;
    cfg.gridCols = 60;
    cfg.gridRows = 45;
    cfg.cellSize = 16;
    
    SDL_Rect gridRect{GRID_MARGIN, GRID_MARGIN, cfg.gridCols * cfg.cellSize, cfg.gridRows * cfg.cellSize};
    SimulationEngine sim(cfg);
    
    auto sensor = std::make_unique<SensorDevice>();
    bool sensorOnline = sensor->open();

    Renderer warehouseRenderer(renderer, gridRect, cfg.cellSize, fontLg, fontMd, fontSm);
    UIMode uiMode = UIMode::NORMAL;

    std::vector<std::unique_ptr<Button>> buttons;
    int bx = panelX + 10, bw = PANEL_W - 20, bh = 26, by = 55;

    auto addBtn = [&](const std::string& label, std::function<void()> fn, Color col = Colors::BTN_NORMAL) {
        buttons.push_back(std::make_unique<Button>(bx, by, bw, bh, label, fn, col));
        by += bh + 4;
        return buttons.back().get();
    };

    Button* btnStart = addBtn("START", [&]{
        std::lock_guard<std::mutex> lock(sim.stateMutex);
        if (sim.state() == SimState::RUNNING) sim.pause();
        else if (sim.state() == SimState::PAUSED) sim.resume();
        else sim.start();
    }, {50,160,90});

    addBtn("RESET", [&]{
        std::lock_guard<std::mutex> lock(sim.stateMutex);
        sim.reset();
        uiMode = UIMode::NORMAL;
    }, {160,80,50});

    addBtn("Generate Warehouse", [&]{
        std::lock_guard<std::mutex> lock(sim.stateMutex);
        sim.loadDefaultMap();
    });

    addBtn("Assign Task", [&]{
        uiMode = UIMode::ASSIGN_TASK;
    });

    by += 4;
    addBtn("Debug LiDAR: OFF", [&]{
        sim.debugMode = !sim.debugMode;
        buttons[4]->setLabel(sim.debugMode ? "Debug LiDAR: ON" : "Debug LiDAR: OFF");
        buttons[4]->setToggled(sim.debugMode);
    });

    std::atomic<bool> threadsRunning{true};
    std::thread simThread(simulationThread, &sim, std::ref(threadsRunning));
    std::thread senThread(sensorThread, &sim, sensor.get(), std::ref(threadsRunning));

    using Clk = std::chrono::steady_clock;
    auto frameStart = Clk::now(), fpsTimer = Clk::now();
    float pulseT = 0.0f, fps = 60.0f;
    int frameCount = 0;

    int taskPickupC = -1, taskPickupR = -1;

    while (!SignalHandler::shutdownRequested()) {
        auto now = Clk::now();
        float dt = static_cast<float>(std::chrono::duration<double>(now - frameStart).count());
        frameStart = now;
        pulseT += dt;
        if (std::chrono::duration<double>(now - fpsTimer).count() >= 1.0) {
            fps = frameCount; frameCount = 0; fpsTimer = now;
        } else frameCount++;

        {
            auto st = sim.state();
            if (st == SimState::RUNNING) btnStart->setLabel("PAUSE");
            else if (st == SimState::PAUSED) btnStart->setLabel("RESUME");
            else btnStart->setLabel("START");
        }

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) SignalHandler::shutdownRequested().store(true);
            for (auto& btn : buttons) btn->handleEvent(event);

            if (event.type == SDL_MOUSEBUTTONDOWN) {
                int mx = event.button.x, my = event.button.y;
                if (mx >= gridRect.x && mx < gridRect.x + gridRect.w && my >= gridRect.y && my < gridRect.y + gridRect.h) {
                    int col = (mx - gridRect.x) / cfg.cellSize;
                    int row = (my - gridRect.y) / cfg.cellSize;
                    std::lock_guard<std::mutex> lock(sim.stateMutex);
                    
                    if (uiMode == UIMode::ASSIGN_TASK) {
                        if (taskPickupC == -1) {
                            taskPickupC = col; taskPickupR = row;
                        } else {
                            sim.assignTask(taskPickupC, taskPickupR, col, row);
                            taskPickupC = -1; taskPickupR = -1;
                            uiMode = UIMode::NORMAL;
                        }
                    } else if (event.button.button == SDL_BUTTON_LEFT) {
                        sim.setObstacle(col, row, true);
                    } else if (event.button.button == SDL_BUTTON_RIGHT) {
                        sim.removeObstacle(col, row);
                    }
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(sim.stateMutex);
            warehouseRenderer.render(sim, pulseT, dt);
        }
        drawPanel(renderer, fontLg, fontMd, fontSm, sim, buttons, panelX, WINDOW_H, uiMode, sensorOnline, fps, sensor.get());
        SDL_RenderPresent(renderer);
    }

    threadsRunning = false;
    simThread.join(); senThread.join();
    sensor->close();
    SDL_DestroyRenderer(renderer); SDL_DestroyWindow(window);
    TTF_Quit(); SDL_Quit();
    return 0;
}
