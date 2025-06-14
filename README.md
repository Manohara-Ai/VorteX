# Vortex Gazebo Simulation with TurtleBot3

This package sets up a custom Gazebo world and spawns a TurtleBot3 robot model for simulation using ROS 2 (Humble).

---

## Prerequisites

Ensure the following are installed:
* ROS 2 Humble
* Gazebo (compatible with Humble)
* `turtlebot3_gazebo` and `gazebo_ros` packages
* NVIDIA GPU (for GPU-accelerated simulation—optional)

---

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
```

---

## World Information

This simulation utilizes a custom Gazebo world. The world is based on the **AWS RoboMaker Racetrack World**, which can be found on their GitHub repository: [https://github.com/aws-robotics/aws-robomaker-racetrack-world](https://github.com/aws-robotics/aws-robomaker-racetrack-world).

---

## Modifying the Camera's Horizontal Field of View (FoV)

To widen the field of view of your TurtleBot3's camera in the simulation, you'll need to adjust the `horizontal_fov` parameter within the robot's URDF (Unified Robot Description Format) or XACRO (XML Macros for ROS) file. This value is typically given in **radians**.

### Steps

1.  **Locate the TurtleBot3 Model Definition:**
    The camera definition for the TurtleBot3 is usually found within the `turtlebot3_description` package. You'll need to pinpoint the specific XACRO file that defines the camera sensor. Common locations include:
    * `/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_waffle.urdf.xacro` (or `turtlebot3_burger.urdf.xacro` if you're using the Burger model).
    * Alternatively, if your `vortex` package has its own custom TurtleBot3 definition, check there. You can often find the exact path by inspecting the `launch` file that spawns the TurtleBot3 in your simulation.

2.  **Edit the `horizontal_fov` Parameter:**
    Open the relevant XACRO file using a text editor (e.g., `gedit`, `nano`, `VS Code`). Search for the `<sensor>` tag related to the camera. Inside this tag, you'll find a `<camera>` block, and within that, a `<horizontal_fov>` tag.

    Change the value within the `<horizontal_fov>` tag from its current value (e.g., `1.02974` radians, which is approximately 59 degrees) to `2.61799` radians (approximately 150 degrees).

    **Example Snippet (what you're looking for and how to change it):**

    ```xml
    <sensor name="camera" type="camera">
        <camera>
            <horizontal_fov>1.02974</horizontal_fov>
            </camera>
        </sensor>

    <sensor name="camera" type="camera">
        <camera>
            <horizontal_fov>2.61799</horizontal_fov>
            </camera>
        </sensor>
    ```

    **Important Considerations:**

    * **Backup:** Always make a **backup of the original file** before making any changes.
    * **Permissions:** If you've installed ROS 2 via `apt`, the files in `/opt/ros/humble/` are typically read-only. You might need to **copy the relevant `turtlebot3_description` package to your workspace**, modify it there, and ensure your `GAZEBO_MODEL_PATH` (or other ROS package paths) points to your modified version first. A common and recommended approach is to **overlay the package in your `ros2_ws/src` directory**. This means copying the entire `turtlebot3_description` folder into `~/ros2_ws/src/` and then making your edits.

3.  **Rebuild your Workspace (if necessary):**
    If you copied and modified the `turtlebot3_description` package in your workspace, you **must rebuild your workspace** for the changes to take effect:

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select turtlebot3_description # Or the name of your modified package
    source install/setup.bash
    ```

---

## Running the Simulation without GPU Acceleration

Gazebo often tries to use GPU acceleration for better performance. However, if you don't have an NVIDIA GPU, or if you encounter issues with GPU acceleration, you can disable it.

The method for disabling GPU acceleration depends on how Gazebo is launched. This is typically controlled by environment variables or specific arguments passed to the Gazebo executable.

### Method 1: Using `GAZEBO_ARGS` Environment Variable (Recommended for this package)

Your `vortex` package likely uses a `launch` file to start Gazebo. This launch file might pass arguments to Gazebo or rely on environment variables. You can often disable GPU acceleration by setting the `GAZEBO_ARGS` environment variable **before** launching your simulation.

To disable GPU acceleration, set `GAZEBO_ARGS` to include `--disable-gpu`:

```bash
export GAZEBO_ARGS="--disable-gpu"
# Then proceed to launch your simulation as usual, for example:
# ros2 launch vortex vortex_simulation.launch.py
```

When Gazebo starts, it checks the `GAZEBO_ARGS` environment variable for additional command-line arguments. By setting `--disable-gpu` here, you're instructing Gazebo to avoid using the GPU for rendering and physics, forcing it to use CPU-based alternatives.

### Method 2: Modifying the Launch File (If Method 1 doesn't work)

If setting `GAZEBO_ARGS` doesn't disable GPU acceleration, the launch file in your `vortex` package might be explicitly overriding it or not passing `GAZEBO_ARGS` correctly. In this case, you would need to modify the launch file itself.

1.  **Locate the Launch File:**
    Find the launch file responsible for starting Gazebo in your `vortex` package. It's typically located in `vortex/launch/` and might be named something like `vortex_simulation.launch.py` or similar.

2.  **Modify the Gazebo Node:**
    Open the launch file and locate the `Node` definition for Gazebo. You'll need to add an argument to disable GPU. The exact parameter might vary slightly depending on the Gazebo version and how the launch file is structured. Look for a section where arguments are passed to the `gzserver` (Gazebo server) or `gzclient` (Gazebo client) executables.

    **Example of a potential modification in a Python launch file:**

    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from launch.actions import ExecuteProcess
    import os

    def generate_launch_description():
        gazebo_ros_package = 'gazebo_ros'
        gazebo_client = 'gzclient'
        gazebo_server = 'gzserver'

        # ... other launch configurations

        # Original Gazebo server launch might look something like this:
        # start_gazebo_server = ExecuteProcess(
        #     cmd=['ros2', 'run', gazebo_ros_package, gazebo_server, '-s', 'libgazebo_ros_factory.so', world_path],
        #     output='screen'
        # )

        # Modified Gazebo server launch to disable GPU:
        start_gazebo_server = ExecuteProcess(
            cmd=['ros2', 'run', gazebo_ros_package, gazebo_server, '--disable-gpu', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'
        )

        # Original Gazebo client launch (might need similar modification):
        # start_gazebo_client = ExecuteProcess(
        #     cmd=['ros2', 'run', gazebo_ros_package, gazebo_client],
        #     output='screen'
        # )

        # Modified Gazebo client launch to disable GPU (if applicable):
        start_gazebo_client = ExecuteProcess(
            cmd=['ros2', 'run', gazebo_ros_package, gazebo_client, '--disable-gpu'],
            output='screen'
        )

        return LaunchDescription([
            # ... other nodes
            start_gazebo_server,
            start_gazebo_client, # If client is launched separately
            # ...
        ])
    ```

    **Note:** The exact `cmd` and arguments might differ. The key is to find where `gzserver` and/or `gzclient` are called and insert `--disable-gpu` into their arguments list.

By following these steps, you can customize the camera's field of view for a wider perspective and ensure your Gazebo simulation runs smoothly even without a dedicated NVIDIA GPU.
