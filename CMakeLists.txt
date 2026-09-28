cmake_minimum_required(VERSION 3.16)
project(WarehouseRobot
    VERSION 2.0.0
    DESCRIPTION "Autonomous Warehouse Robot Pathfinding & Obstacle Simulator"
    LANGUAGES CXX)

# ── C++ standard ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Compiler warnings ─────────────────────────────────────────────────────────
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -Wall -Wextra -Wpedantic
        -Wno-unused-function
    )
endif()

# ── Build type ────────────────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# ── Find SDL2 ────────────────────────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2       REQUIRED sdl2)
pkg_check_modules(SDL2_TTF   REQUIRED SDL2_ttf)

message(STATUS "SDL2 version: ${SDL2_VERSION}")
message(STATUS "SDL2_ttf version: ${SDL2_TTF_VERSION}")

# ── Find Threads ──────────────────────────────────────────────────────────────
find_package(Threads REQUIRED)

# ── Include directories ───────────────────────────────────────────────────────
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${SDL2_INCLUDE_DIRS}
    ${SDL2_TTF_INCLUDE_DIRS}
)

# ── Core library ─────────────────────────────────────────────────────────────
add_library(warehouse_core STATIC
    src/core/Grid.cpp
    src/core/AStarPlanner.cpp
    src/core/Robot.cpp
    src/core/DynamicObstacle.cpp
    src/core/SimulationEngine.cpp
    src/core/Metrics.cpp
)
target_include_directories(warehouse_core PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_features(warehouse_core PUBLIC cxx_std_17)

# ── System library ────────────────────────────────────────────────────────────
add_library(warehouse_system STATIC
    src/system/Logger.cpp
    src/system/SignalHandler.cpp
)
target_include_directories(warehouse_system PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(warehouse_system PUBLIC Threads::Threads)

# ── Driver interface library ──────────────────────────────────────────────────
add_library(warehouse_driver STATIC
    src/driver/SensorDevice.cpp
)
target_include_directories(warehouse_driver PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/driver   # for warehouse_sensor_ioctl.h
)
target_link_libraries(warehouse_driver PUBLIC warehouse_system)

# ── UI library ────────────────────────────────────────────────────────────────
add_library(warehouse_ui STATIC
    src/ui/Renderer.cpp
    src/ui/Button.cpp
)
target_include_directories(warehouse_ui PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${SDL2_INCLUDE_DIRS}
    ${SDL2_TTF_INCLUDE_DIRS}
)
target_link_libraries(warehouse_ui PUBLIC warehouse_core)

# ── Main executable ───────────────────────────────────────────────────────────
add_executable(warehouse_robot app/main.cpp)
target_include_directories(warehouse_robot PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${SDL2_INCLUDE_DIRS}
    ${SDL2_TTF_INCLUDE_DIRS}
)
target_link_libraries(warehouse_robot PRIVATE
    warehouse_core
    warehouse_system
    warehouse_ui
    warehouse_driver
    ${SDL2_LIBRARIES}
    ${SDL2_TTF_LIBRARIES}
    Threads::Threads
)

# ── Enable CTest ──────────────────────────────────────────────────────────────
enable_testing()

# ── Test executables ─────────────────────────────────────────────────────────
set(TEST_LIBS warehouse_core warehouse_system)

add_executable(test_astar        tests/test_astar.cpp)
add_executable(test_grid         tests/test_grid.cpp)
add_executable(test_robot        tests/test_robot.cpp)
add_executable(test_collision    tests/test_collision.cpp)
add_executable(test_sensor_iface tests/test_sensor_interface.cpp)

foreach(TEST_NAME test_astar test_grid test_robot test_collision test_sensor_iface)
    target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)
    target_link_libraries(${TEST_NAME} PRIVATE ${TEST_LIBS})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()

target_link_libraries(test_sensor_iface PRIVATE warehouse_driver)

# ── Utility tools ─────────────────────────────────────────────────────────────
add_executable(sensor_test tools/sensor_test.cpp)
target_include_directories(sensor_test PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/driver)
target_link_libraries(sensor_test PRIVATE warehouse_driver warehouse_system)

add_executable(warehouse_cli tools/warehouse_cli.cpp)
target_include_directories(warehouse_cli PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(warehouse_cli PRIVATE warehouse_core warehouse_system)

# ── Create logs directory ─────────────────────────────────────────────────────
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/logs)

# ── Installation ─────────────────────────────────────────────────────────────
install(TARGETS warehouse_robot DESTINATION bin)
install(TARGETS sensor_test warehouse_cli DESTINATION bin)

# ── Summary ───────────────────────────────────────────────────────────────────
message(STATUS "")
message(STATUS "=== Warehouse Robot Build Configuration ===")
message(STATUS "  Build type:  ${CMAKE_BUILD_TYPE}")
message(STATUS "  Compiler:    ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "  C++ std:     17")
message(STATUS "  SDL2:        ${SDL2_VERSION}")
message(STATUS "  Install dir: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "===========================================")
message(STATUS "")
message(STATUS "Build: cmake --build build")
message(STATUS "Run:   ./build/warehouse_robot")
message(STATUS "Test:  ctest --test-dir build -V")
