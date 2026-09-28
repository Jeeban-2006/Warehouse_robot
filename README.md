# Autonomous Warehouse Robot Pathfinding & Obstacle Simulator

A complete Linux-based C++ simulation of an autonomous warehouse robot, featuring a custom A* pathfinding engine, dynamic obstacle avoidance, thread-safe synchronization, and a simulated Linux kernel character device driver.

This project was developed for the **Embedded Systems / Linux training program**.

## Features

- **Custom A* Pathfinding**: Implemented entirely from scratch in C++ (no external routing libraries). Features priority queues, heuristic optimization, and path validity checking.
- **Dynamic Obstacle Avoidance**: Dynamic obstacles bounce along predefined axes. The robot's simulation engine replans the path automatically if the current route becomes blocked.
- **System Programming**: Utilizes POSIX/C++17 threading, mutexes, condition variables, atomic types, and signal handling for a smooth, lock-safe multi-threaded simulation.
- **Linux Device Driver Interface**: A custom C kernel module (`warehouse_sensor_driver.c`) acts as a virtual character device (`/dev/warehouse_sensor`). The C++ application communicates via `ioctl`, `read`, `write`, and `poll`.
- **SDL2 Visualization**: A fully custom-built renderer and UI using SDL2 and SDL2_ttf.
- **Thread Architecture**:
  1.  **Main (UI) Thread**: SDL2 event polling, rendering loop.
  2.  **Simulation Thread**: Updates robot kinematics and grid state at a steady 60Hz.
  3.  **Sensor Thread**: Translates simulation events into driver payloads and syncs with the kernel module.

## Architecture

```
App -> SimulationEngine -> Grid / Robot / DynamicObstacles / AStarPlanner
          |
          v
      SensorDevice (C++ Wrapper)
          |
         poll() / ioctl() / read() / write()
          |
[ Linux Kernel Mode ] -> /dev/warehouse_sensor (Miscdevice Driver)
```

## Prerequisites (Linux / WSL2 Ubuntu)

Install dependencies:
```bash
sudo apt-get update
sudo apt-get install build-essential cmake libsdl2-dev libsdl2-ttf-dev libsdl2-image-dev pkg-config git linux-headers-$(uname -r)
```

*(Note: On WSL2 custom Microsoft kernels, `linux-headers` might not be available via apt. In that case, the C++ application gracefully falls back to a simulated sensor mode).*

## Building

```bash
cd warehouse_robot
mkdir build && cd build
cmake ..
cmake --build . -j$(nproc)
```

## Running the Simulator

```bash
./build/warehouse_robot
```

### Controls:
- **SPACE**: Start / Pause
- **R**: Reset Simulation
- **F**: Find Path (without moving)
- **S / G**: Enter "Set Start" or "Set Goal" mode (Click on grid)
- **Left Click**: Place static obstacle
- **Right Click**: Remove static obstacle
- **D**: Toggle Debug Overlay

## Running Tests

```bash
cd build
ctest --output-on-failure
```
Or run the headless CLI test:
```bash
./build/warehouse_cli
```

## Testing the Kernel Driver

If you are on a native Linux environment with matching kernel headers:

1. Build and load the driver:
```bash
cd driver
make
sudo make load
```
2. Verify the device:
```bash
ls -l /dev/warehouse_sensor
```
3. Run the driver test tool:
```bash
../build/sensor_test
```
4. Unload:
```bash
sudo make unload
```

## Documentation

Full documentation is available in the `docs/` folder, structured across the 6 project stages:
- **Stage 1**: Introduction, Checklists
- **Stage 2**: Requirements (PRD, FR, NFR, Use Cases)
- **Stage 3**: Architecture, Component Design, UML
- **Stage 4**: Prototype progress
- **Stage 5**: Test Results, Metrics, Code Quality
- **Stage 6**: Final Report, Viva Questions, Demo Script
