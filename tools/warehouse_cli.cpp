// warehouse_cli.cpp - Command-line simulation tool (headless, no SDL2).
// Tests the core simulation engine without a GUI window.
#include "core/SimulationEngine.hpp"
#include "core/Robot.hpp"
#include "system/Logger.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>
#include <iomanip>

using namespace warehouse;

int main(int argc, char* argv[]) {
    Logger::instance().init("", LogLevel::INFO);

    std::cout << "==========================================\n";
    std::cout << "  Warehouse Robot CLI (Headless Test)\n";
    std::cout << "==========================================\n\n";

    SimConfig cfg;
    cfg.gridCols  = 30;
    cfg.gridRows  = 22;
    cfg.cellSize  = 32;

    SimulationEngine sim(cfg);
    sim.dynamicObsEnabled = true;
    sim.speedMultiplier   = 3.0f;   // Run fast

    std::cout << "[INFO] Warehouse: " << cfg.gridCols << "x" << cfg.gridRows
              << " cell=" << cfg.cellSize << "\n";
    std::cout << "[INFO] Robot: (" << sim.robot().col() << "," << sim.robot().row() << ")"
              << " -> (" << sim.robot().goalPos().first << "," << sim.robot().goalPos().second << ")\n";
    std::cout << "[INFO] Obstacles: " << sim.grid().obstacleCount() << "\n";
    std::cout << "[INFO] DynObs: " << sim.dynObs().size() << "\n\n";

    // Start simulation
    sim.start();
    std::cout << "[INFO] Start: " << sim.statusMessage() << "\n";
    std::cout << "[INFO] State: " << simStateStr(sim.state()) << "\n\n";

    if (sim.state() == SimState::NO_PATH) {
        std::cout << "[FAIL] No path found.\n";
        return 1;
    }

    std::cout << "[INFO] Path: " << sim.metrics().lastPathLength << " cells, "
              << sim.metrics().lastNodesExplored << " explored, "
              << std::fixed << std::setprecision(2)
              << sim.metrics().lastPlanTimeMs << " ms\n\n";

    // Simulate at 60Hz for up to 30 real seconds
    int steps = 0;
    auto wallStart = std::chrono::steady_clock::now();
    constexpr float SIM_DT = 1.0f / 60.0f;

    while (sim.state() == SimState::RUNNING && steps < 10000) {
        sim.update(SIM_DT);
        ++steps;

        // Print status every 120 frames (2 sim-seconds)
        if (steps % 120 == 0) {
            auto& r = sim.robot();
            std::cout << "[T=" << std::fixed << std::setprecision(1)
                      << (steps * SIM_DT) << "s] "
                      << "Robot(" << r.col() << "," << r.row() << ") "
                      << robotStateStr(r.state())
                      << " replans=" << sim.metrics().replanCount << "\n";
        }
    }

    auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - wallStart).count();

    std::cout << "\n[INFO] Simulation finished in " << steps << " steps ("
              << std::fixed << std::setprecision(2) << elapsed << "s wall time)\n";
    std::cout << "[INFO] Final state: " << simStateStr(sim.state()) << "\n";
    std::cout << "[INFO] " << sim.statusMessage() << "\n\n";
    std::cout << "[METRICS] " << sim.metrics().summary() << "\n";

    if (sim.state() == SimState::COMPLETED) {
        std::cout << "\n=== SIMULATION: PASSED ===\n";
        return 0;
    } else {
        std::cout << "\n=== SIMULATION: Did not complete (state=" << simStateStr(sim.state()) << ") ===\n";
        return 1;
    }
}
