# Project Introduction
## 1. Project Title
Autonomous Warehouse Robot Pathfinding & Obstacle Simulator

## 2. Background
Automated warehouse navigation requires advanced pathfinding.

## 3. Problem Statement
Robots must navigate dynamically changing grids efficiently.

## 4. Motivation
To build a scalable C++ simulator demonstrating A* and Linux drivers.

## 5. Proposed Solution
A 30x22 grid simulation in C++17 with SDL2 and multithreading.

## 6. Project Objectives
1. Implement A*
2. Develop SDL2 GUI
3. Linux character driver
4. Multithreading
5. Dynamic obstacle avoidance
6. IPC & sync
7. File descriptors/poll
8. C++17 standards
9. Comprehensive testing
10. Architectural docs

## 7. Scope
Single robot, dynamic obstacles, Linux kernel module.

## 8. Out of Scope
Real hardware, 3D simulation.

## 9. Expected Outcome
Robust C++ simulator.

## 10. Applications
Warehouse logistics, robotics education.

## 11. Key Technologies
C++17, SDL2, Linux Kernel.

## 12. Success Criteria
Real-time navigation, no memory leaks.

## 13. Risks and Mitigations
WSL limitation -> Mock driver fallback.

## 14. Assumptions
Running on WSL2 Ubuntu.
