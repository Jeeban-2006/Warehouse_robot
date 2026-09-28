#!/usr/bin/env bash
# clean.sh - Remove build artifacts.
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
echo "Cleaning build..."
rm -rf "$PROJECT_DIR/build"
echo "Clean complete."
