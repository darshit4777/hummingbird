#!/bin/bash

# run_visualizer.sh
# Convenience script to run the IMU Visualizer
#
# Usage:
#   ./run_visualizer.sh [PORT]
#
# Examples:
#   ./run_visualizer.sh                # Use default port (can select in GUI)
#   ./run_visualizer.sh /dev/ttyUSB0   # Pre-select specific port

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   IMU Visualizer - MPU6050${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if Python 3 is installed
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}Error: Python 3 is not installed${NC}"
    echo "Please install Python 3 and try again"
    exit 1
fi

# Check if virtual environment exists
if [ ! -d "${SCRIPT_DIR}/venv" ]; then
    echo -e "${YELLOW}Virtual environment not found. Creating...${NC}"
    python3 -m venv "${SCRIPT_DIR}/venv"
    echo -e "${GREEN}✓ Virtual environment created${NC}"
    echo ""
fi

# Activate virtual environment
source "${SCRIPT_DIR}/venv/bin/activate"

# Check if dependencies are installed
if [ ! -f "${SCRIPT_DIR}/venv/.dependencies_installed" ]; then
    echo -e "${YELLOW}Installing dependencies...${NC}"
    pip install --upgrade pip > /dev/null
    pip install -r "${SCRIPT_DIR}/requirements.txt"

    if [ $? -eq 0 ]; then
        touch "${SCRIPT_DIR}/venv/.dependencies_installed"
        echo -e "${GREEN}✓ Dependencies installed${NC}"
        echo ""
    else
        echo -e "${RED}✗ Failed to install dependencies${NC}"
        exit 1
    fi
fi

# Run the visualizer
echo -e "${GREEN}Starting IMU Visualizer...${NC}"
echo ""

cd "${SCRIPT_DIR}"
python3 ImuVisualizer.py "$@"
