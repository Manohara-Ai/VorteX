# VorteX Gazebo Simulation with TurtleBot3

This ROS 2 (Humble) package sets up a TurtleBot3 robot in a custom Gazebo world for simulation. It includes automatic setup, camera field-of-view modifications, and GPU detection to streamline the simulation experience.

---

## Quick Setup

1. Clone this repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Manohara-Ai/VorteX.git vortex
````

2. Run the one-step setup script:

```bash
cd ~/ros2_ws/src/vortex
chmod +x setup.sh
./setup.sh
```

> The script installs dependencies, patches camera settings, detects GPU, and builds the workspace.

> ✅ Tested on Ubuntu 22.04 + ROS 2 Humble.

---

## Directory Structure

```
vortex/
├── launch/            # Launch files for Gazebo, RViz, etc.
├── models/            # Custom Gazebo models
├── worlds/            # Custom simulation world(s)
├── rviz/              # RViz configuration files
├── test/              # Test assets (if applicable)
├── vortex/            # (Optional) Python package or scripts
├── setup.sh           # One-step setup script
├── package.xml        # ROS 2 package metadata
├── setup.py / cfg     # Python packaging (if used)
├── LICENSE            # MIT License
└── README.md          # You're here!
```

---

## Features

* One-line setup via `setup.sh`
* Custom world based on AWS RoboMaker Racetrack
* Automatically expands TurtleBot3 camera FoV to 150°
* Detects NVIDIA GPU and configures Gazebo accordingly
* ROS 2 Humble-compatible

---

## Requirements (Auto-installed)

* ROS 2 Humble
* `turtlebot3_gazebo`, `gazebo_ros`, and `teleop-twist-keyboard`
* Gazebo Classic (compatible with ROS 2)
* Optional: NVIDIA GPU (used if available)

---

## Running the Simulation

After setup is complete:

```bash
# Source ROS 2 and your workspace
source ~/.bashrc

# Launch the Gazebo simulation
ros2 launch vortex world.launch.py

# Optional: RViz
ros2 launch vortex rviz.launch.py

# Optional: Autonomous behavior
ros2 launch vortex vortex.launch.py
```

---

## Camera Field of View

The `setup.sh` script modifies TurtleBot3’s camera `horizontal_fov` to `2.61799` radians (\~150°) in:

* `model.sdf`
* `model-1_4.sdf`

Path:

```
/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/
```

If already set, the script will skip modification.

---

## No GPU? No Problem.

If no NVIDIA GPU is found:

* GPU offload in the launch file is automatically disabled.

If you want to **force-disable** GPU manually:

```bash
export GAZEBO_ARGS="--disable-gpu"
```

---

## Rebuilding the Workspace

After any changes:

```bash
cd ~/ros2_ws
colcon build --packages-select vortex
source install/setup.bash
```

---

## Acknowledgements

* AWS RoboMaker Racetrack World: [github.com/aws-robotics/aws-robomaker-racetrack-world](https://github.com/aws-robotics/aws-robomaker-racetrack-world)
* TurtleBot3 Simulation by ROBOTIS

---

## Collaboration & Contributions

We are open to collaborations and improvements!

If you'd like to contribute:

1. Fork this repository
2. Create a new branch (`git checkout -b your-feature-name`)
3. Make your changes
4. Submit a pull request (PR)

We welcome:
- New launch configurations
- Improved robot behavior or navigation logic
- Additional worlds or test cases
- Performance or compatibility fixes

Your contributions are greatly appreciated! 

---

## License

This project is licensed under the [MIT License](./LICENSE).
