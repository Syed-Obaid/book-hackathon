#!/bin/bash
# ROS 2 Humble Installation Script for Ubuntu 22.04
# Reference: Chapter 1.2 Installation & Workspace Setup
# Tested on: Ubuntu 22.04 LTS (Jammy Jellyfish)

set -e  # Exit on error

echo "========================================="
echo "ROS 2 Humble Installation Script"
echo "Ubuntu 22.04 LTS"
echo "========================================="

# Check Ubuntu version
if [ "$(lsb_release -cs)" != "jammy" ]; then
    echo "ERROR: This script requires Ubuntu 22.04 (Jammy Jellyfish)"
    echo "Current version: $(lsb_release -ds)"
    exit 1
fi

echo ""
echo "[1/4] Setting up ROS 2 repository sources..."

# Ensure universe repository is enabled
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

# Install curl for downloading GPG key
sudo apt update
sudo apt install -y curl

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "✓ ROS 2 repository configured"

echo ""
echo "[2/4] Installing ROS 2 Humble Desktop..."
echo "(This may take 10-15 minutes and download ~1.5GB)"

# Update package index
sudo apt update

# Install ROS 2 Humble Desktop (includes RViz, rqt, demos)
sudo apt install -y ros-humble-desktop python3-argcomplete

echo "✓ ROS 2 Humble Desktop installed"

echo ""
echo "[3/4] Installing development tools..."

# Install colcon and development tools
sudo apt install -y ros-dev-tools python3-colcon-common-extensions

echo "✓ Development tools installed"

echo ""
echo "[4/4] Verifying installation..."

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Check if ros2 command is available
if command -v ros2 &> /dev/null; then
    echo "✓ ros2 command available"
    echo ""
    echo "ROS 2 version:"
    ros2 --version
else
    echo "✗ ERROR: ros2 command not found"
    exit 1
fi

echo ""
echo "========================================="
echo "Installation Complete!"
echo "========================================="
echo ""
echo "Next steps:"
echo "1. Add ROS 2 to your shell startup:"
echo "   echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"
echo "   source ~/.bashrc"
echo ""
echo "2. Create a workspace:"
echo "   mkdir -p ~/ros2_ws/src"
echo "   cd ~/ros2_ws"
echo "   colcon build --symlink-install"
echo ""
echo "3. Test with demo nodes:"
echo "   ros2 run demo_nodes_cpp talker"
echo ""
echo "For more details, see Chapter 1.2 Installation & Workspace Setup"
echo "========================================="
