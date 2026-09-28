#!/usr/bin/env bash
# build.sh - Build the warehouse robot simulator.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
BUILD_TYPE="${1:-Release}"

echo "============================================"
echo "  Building Warehouse Robot Simulator"
echo "  Type: $BUILD_TYPE"
echo "  Dir:  $BUILD_DIR"
echo "============================================"

# Create logs directory
mkdir -p "$PROJECT_DIR/logs"

# Configure
cmake -S "$PROJECT_DIR" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Build
cmake --build "$BUILD_DIR" --parallel "$(nproc)"

echo ""
echo "============================================"
echo "  Build complete."
echo "  Run:  $BUILD_DIR/warehouse_robot"
echo "  Test: cd $BUILD_DIR && ctest -V"
echo "============================================"
