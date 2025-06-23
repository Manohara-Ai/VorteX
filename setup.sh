#!/bin/bash

set -e

# ---------------
# COLORS & STYLES
# ---------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

function info()    { echo -e "${CYAN}[INFO]${NC} $1"; }
function success() { echo -e "${GREEN}[OK]${NC} $1"; }
function warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
function error()   { echo -e "${RED}[ERROR]${NC} $1"; }

# ------------------------------
# Banner
# ------------------------------
echo -e "${CYAN}"
echo "=============================================="
echo "           VorteX ROS 2 Setup Script"
echo "=============================================="
echo -e "${NC}"

# ------------------------------
# Step 1: Install system tools
# ------------------------------
info "[Step 1] Installing system tools..."
if ! command -v gedit &> /dev/null; then
    sudo apt update
    sudo apt install -y gedit curl git build-essential
    success "System tools installed."
else
    success "gedit and system tools already installed. Skipping."
fi

# ------------------------------
# Step 2: Install Gazebo Classic & TurtleBot3
# ------------------------------
info "[Step 2] Installing Gazebo Classic and TurtleBot3..."
if ! dpkg -l | grep -q ros-humble-turtlebot3-gazebo; then
    sudo apt install -y ros-humble-gazebo* \
                        ros-humble-turtlebot3-gazebo \
                        ros-humble-turtlebot3-msgs \
                        ros-humble-turtlebot3-description \
                        ros-humble-turtlebot3-bringup \
                        ros-humble-teleop-twist-keyboard
    success "Gazebo and TurtleBot3 packages installed."
else
    success "Gazebo and TurtleBot3 already installed. Skipping."
fi

# ------------------------------
# Step 3: Setup environment variables
# ------------------------------
info "[Step 3] Setting up environment variables..."

function append_if_missing() {
    local LINE="$1"
    grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc
}

append_if_missing "export TURTLEBOT3_MODEL=waffle"
append_if_missing "source /opt/ros/humble/setup.bash"
append_if_missing "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
append_if_missing "source ~/ros2_ws/install/setup.bash"

GAZEBO_PATH_LINE='export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix vortex)/share/vortex/models:$(ros2 pkg prefix vortex)/share/vortex/worlds:/opt/ros/humble/share/turtlebot3_gazebo/models'
append_if_missing "$GAZEBO_PATH_LINE"

source ~/.bashrc
success "Environment variables set."

# ------------------------------
# Step 4: Modify TurtleBot3 Camera FOV
# ------------------------------
info "[Step 4] Modifying TurtleBot3 Camera FOV..."
MODEL_DIR="/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle"

for file in model.sdf model-1_4.sdf; do
    TARGET="$MODEL_DIR/$file"
    if [ -f "$TARGET" ]; then
        if grep -q "<horizontal_fov>2.61799</horizontal_fov>" "$TARGET"; then
            success "FOV in $file already set correctly. No change needed."
        elif grep -q "<horizontal_fov>" "$TARGET"; then
            sudo sed -i 's|<horizontal_fov>.*</horizontal_fov>|<horizontal_fov>2.61799</horizontal_fov>|g' "$TARGET"
            success "Updated FOV in $file."
        else
            warn "$file exists but does not contain a <horizontal_fov> tag. Skipping."
        fi
    else
        warn "$file not found. Skipping."
    fi
done

# ------------------------------
# Step 5: GPU Detection & Launch File
# ------------------------------
info "[Step 5] Checking for GPU and updating launch file..."
LAUNCH_FILE=$(realpath ./launch/world.launch.py)

if [ -f "$LAUNCH_FILE" ]; then
    if ! lspci | grep -i nvidia &> /dev/null; then
        sed -i '/cmd=\[/,/gzclient\]/c\    cmd=[\n        '\''gzclient'\''\n    ]' "$LAUNCH_FILE"
        success "No NVIDIA GPU found. Disabled GPU offload."
    else
        success "NVIDIA GPU found. Keeping GPU settings."
    fi
else
    warn "Launch file not found. Skipping GPU config."
fi

# ------------------------------
# Step 6: Download DepthAnything model
# ------------------------------
info "[Step 6] Downloading DepthAnythingV2 model..."
CHECKPOINT_DIR="./checkpoints"
CHECKPOINT_PATH="$CHECKPOINT_DIR/depth_anything_v2_vits.pth"
MODEL_URL="https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth?download=true"

mkdir -p "$CHECKPOINT_DIR"

if [ ! -f "$CHECKPOINT_PATH" ]; then
    info "Downloading model..."
    curl -L "$MODEL_URL" -o "$CHECKPOINT_PATH"
    success "Model downloaded to $CHECKPOINT_PATH."
else
    success "Model already exists. Skipping download."
fi

# ------------------------------
# Step 7: Build Workspace
# ------------------------------
info "[Step 7] Building VorteX workspace..."
cd ~/ros2_ws
if ! colcon list | grep -q vortex; then
    error "VorteX package not found in workspace. Build skipped."
else
    colcon build --packages-select vortex
    source install/setup.bash
    success "Build complete."
fi

# ------------------------------
# Completion Message
# ------------------------------
echo -e "${CYAN}"
echo "----------------------------------------------"
echo " VorteX Simulation Setup Completed Successfully"
echo " To launch the simulation, run:"
echo "     ros2 launch vortex world.launch.py"
echo "----------------------------------------------"
echo -e "${NC}"
