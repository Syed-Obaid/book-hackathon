#!/bin/bash
# ROS 2 Workspace Creation Script
# Reference: Chapter 1.2 Installation & Workspace Setup
# Creates a standard ROS 2 workspace with colcon build system

set -e  # Exit on error

# Default workspace name
WORKSPACE_NAME="${1:-ros2_ws}"

echo "========================================="
echo "ROS 2 Workspace Creation Script"
echo "Workspace: ~/$WORKSPACE_NAME"
echo "========================================="

# Check if ROS 2 is sourced
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo "WARNING: ROS 2 environment not sourced"
    echo "Sourcing /opt/ros/humble/setup.bash..."
    source /opt/ros/humble/setup.bash
fi

echo ""
echo "[1/3] Creating workspace directory structure..."

# Create workspace directories
cd ~
mkdir -p "$WORKSPACE_NAME/src"

echo "✓ Created ~/$WORKSPACE_NAME/"
echo "  ├── src/        (source code)"
echo "  ├── build/      (will be created by colcon)"
echo "  ├── install/    (will be created by colcon)"
echo "  └── log/        (will be created by colcon)"

echo ""
echo "[2/3] Building workspace..."

cd "$WORKSPACE_NAME"

# Initial build (creates build/, install/, log/ directories)
colcon build --symlink-install

echo "✓ Workspace built successfully"

echo ""
echo "[3/3] Configuring environment..."

# Check if setup.bash was created
if [ -f "install/setup.bash" ]; then
    echo "✓ install/setup.bash created"
else
    echo "✗ ERROR: install/setup.bash not found"
    exit 1
fi

# Source the workspace
source install/setup.bash

# Verify workspace is active
if echo "$AMENT_PREFIX_PATH" | grep -q "$WORKSPACE_NAME"; then
    echo "✓ Workspace is active in current shell"
else
    echo "✗ WARNING: Workspace not in AMENT_PREFIX_PATH"
fi

echo ""
echo "========================================="
echo "Workspace Creation Complete!"
echo "========================================="
echo ""
echo "Workspace location: ~/$WORKSPACE_NAME"
echo ""
echo "To activate workspace in current terminal:"
echo "  source ~/$WORKSPACE_NAME/install/setup.bash"
echo ""
echo "To auto-activate on terminal startup, add to ~/.bashrc:"
echo "  echo 'source ~/$WORKSPACE_NAME/install/setup.bash' >> ~/.bashrc"
echo ""
echo "Directory structure:"
cd "$WORKSPACE_NAME"
tree -L 1 -d 2>/dev/null || ls -l

echo ""
echo "Next steps:"
echo "1. Add packages to src/ directory"
echo "2. Run 'colcon build' to build packages"
echo "3. Source 'install/setup.bash' to use packages"
echo ""
echo "For more details, see Chapter 1.2 Installation & Workspace Setup"
echo "========================================="
