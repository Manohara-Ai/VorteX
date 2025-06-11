# Vortex Gazebo Simulation with TurtleBot3

This package sets up a custom Gazebo world and spawns a TurtleBot3 robot model for simulation using ROS 2 (Humble).

## Prerequisites

Ensure the following are installed:
- ROS 2 Humble
- Gazebo (compatible with Humble)
- `turtlebot3_gazebo` and `gazebo_ros` packages
- NVIDIA GPU (for GPU-accelerated simulation — optional)

## Environment Setup

Before building and running the simulation, export these environment variables and source the necessary setup files:

```bash
# 1. Set your TurtleBot3 model
export TURTLEBOT3_MODEL=waffle

# 2. Source ROS 2 setup
source /opt/ros/humble/setup.bash

# 3. Enable colcon tab completion (optional)
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# 4. Source your workspace
source ~/ros2_ws/install/setup.bash

# 5. Export custom model and world paths for Gazebo
export GAZEBO_MODEL_PATH=$(ros2 pkg prefix vortex)/share/vortex/models:$(ros2 pkg prefix vortex)/share/vortex/worlds
