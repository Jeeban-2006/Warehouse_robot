#!/usr/bin/env bash
# run.sh - Build and run the warehouse robot simulator.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Build first
"$SCRIPT_DIR/build.sh" Release

echo ""
echo "Starting Warehouse Robot Simulator..."
"$PROJECT_DIR/build/warehouse_robot" "$@"
