#!/bin/bash

# run_reader.sh
# Convenience script to build and run the MPU6050 Reader
#
# Usage:
#   ./run_reader.sh [PORT] [OPTIONS]
#
# Examples:
#   ./run_reader.sh                        # Use default port /dev/ttyACM0
#   ./run_reader.sh /dev/ttyUSB0          # Use specified port
#   ./run_reader.sh --skip-build          # Skip build, just run
#   ./run_reader.sh /dev/ttyACM0 --debug  # Debug build and run

set -e  # Exit on error

# Default values
DEFAULT_PORT="/dev/ttyACM0"
PORT="${DEFAULT_PORT}"
SKIP_BUILD=false
BUILD_CONFIG=""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Parse arguments
for arg in "$@"; do
    case $arg in
        --skip-build)
            SKIP_BUILD=true
            ;;
        --debug)
            BUILD_CONFIG="--config=debug"
            ;;
        --release)
            BUILD_CONFIG="--config=release"
            ;;
        --help|-h)
            echo "MPU6050 Reader - Build and Run Script"
            echo ""
            echo "Usage: $0 [PORT] [OPTIONS]"
            echo ""
            echo "Arguments:"
            echo "  PORT              Serial port (default: /dev/ttyACM0)"
            echo ""
            echo "Options:"
            echo "  --skip-build      Skip the build step, just run existing binary"
            echo "  --debug           Build with debug configuration"
            echo "  --release         Build with release configuration"
            echo "  --help, -h        Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                          # Build and run with default port"
            echo "  $0 /dev/ttyUSB0             # Use USB0 port"
            echo "  $0 --skip-build             # Skip build, run existing binary"
            echo "  $0 /dev/ttyACM0 --debug     # Debug build"
            echo ""
            exit 0
            ;;
        /dev/*)
            PORT="$arg"
            ;;
    esac
done

# Get project root (2 levels up from this script)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/../.." && pwd )"

# Binary path
BINARY_PATH="${PROJECT_ROOT}/bazel-bin/imu/mpu6050/Mpu6050Reader"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   MPU6050 Reader - Build & Run${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Build the binary if not skipping
if [ "$SKIP_BUILD" = false ]; then
    echo -e "${YELLOW}Building Mpu6050Reader...${NC}"
    echo -e "Config: ${BUILD_CONFIG:-default}"
    echo ""

    cd "${PROJECT_ROOT}"

    if bazel build ${BUILD_CONFIG} //imu/mpu6050:Mpu6050Reader; then
        echo ""
        echo -e "${GREEN}✓ Build successful!${NC}"
        echo ""
    else
        echo ""
        echo -e "${RED}✗ Build failed!${NC}"
        exit 1
    fi
else
    echo -e "${YELLOW}Skipping build step...${NC}"
    echo ""
fi

# Check if binary exists
if [ ! -f "${BINARY_PATH}" ]; then
    echo -e "${RED}✗ Binary not found at: ${BINARY_PATH}${NC}"
    echo -e "${YELLOW}Please build the project first (run without --skip-build)${NC}"
    exit 1
fi

# Check if port exists
if [ ! -e "${PORT}" ]; then
    echo -e "${YELLOW}⚠ Warning: Serial port ${PORT} does not exist${NC}"
    echo ""
    echo "Available serial ports:"
    ls -la /dev/tty{USB,ACM}* 2>/dev/null || echo "  No USB/ACM ports found"
    echo ""
    read -p "Continue anyway? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check port permissions
if [ -e "${PORT}" ] && [ ! -r "${PORT}" ]; then
    echo -e "${YELLOW}⚠ Warning: No read permission for ${PORT}${NC}"
    echo ""
    echo "To fix permissions, run:"
    echo "  sudo usermod -a -G dialout \$USER"
    echo "  (then log out and log back in)"
    echo ""
    echo "Or run this script with sudo (not recommended)"
    echo ""
fi

echo -e "${GREEN}Running Mpu6050Reader...${NC}"
echo -e "Port: ${PORT}"
echo -e "Binary: ${BINARY_PATH}"
echo ""
echo -e "${BLUE}----------------------------------------${NC}"
echo ""

# Run the binary
exec "${BINARY_PATH}" "${PORT}"
