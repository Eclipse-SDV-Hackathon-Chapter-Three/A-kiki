#!/bin/bash
#
# This script installs the necessary dependencies for the
# Enhanced Police Monitoring System (Python-only version).
# It assumes CARLA Simulator and ROS 2 Humble are already installed.
#

set -e

echo "--- Starting Dependency Installation ---"

# 1. Install Prerequisite System Packages using APT
echo "[1/3] Installing prerequisite system packages..."
sudo apt-get update
sudo apt-get install -y git curl python3-pip
echo "✅ Prerequisite system packages installed."

# 2. Install Rust (required for zenoh)
echo "[2/3] Installing Rust..."
if ! command -v rustc &> /dev/null
then
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source "$HOME/.cargo/env"
    echo "✅ Rust installed."
else
    echo "✅ Rust is already installed."
fi

# 3. Install Python Packages using PIP
echo "[3/3] Installing Python packages..."

# Install eclipse-zenoh (newer package name)
echo "Installing eclipse-zenoh..."
pip install eclipse-zenoh jsonschema websockets opencv-python

# Install ROS2 Python packages if not already installed
if ! python3 -c "import rclpy" 2>/dev/null; then
    echo "Installing ROS2 Python packages..."
    sudo apt-get install -y python3-rclpy ros-humble-cv-bridge
fi

echo "✅ Python packages installed."

echo "--- Dependency installation complete! ---"
echo ""
echo "🎉 All dependencies installed successfully!"
echo ""
echo "Next steps:"
echo "1. Make sure CARLA Simulator is running on localhost:2000"
echo "2. Source ROS2 environment: source /opt/ros/humble/setup.bash"
echo "3. Run the system: python3 run_enhanced_police_system.py"
echo ""
echo "Note: This system now uses Python-only bridges (no C++ compilation needed)!"