import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('vortex'), 'launch')

    x_pose = LaunchConfiguration('x_pose', default='-1')
    y_pose = LaunchConfiguration('y_pose', default='-15.5')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0')

    package_dir = get_package_share_directory('vortex')

    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator', default_value='True',
        description='Whether to start the simulator'
    )

    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='False',
        description='Run without GUI (gzclient)'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(package_dir, 'worlds', 'racetrack_day.world'),
        description='Path to the world file'
    )

    start_gzserver_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
        cwd=[package_dir],
        output='screen'
    )

    start_gzclient_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        cmd=[
            'env',
            '__NV_PRIME_RENDER_OFFLOAD=1',
            '__GLX_VENDOR_LIBRARY_NAME=nvidia',
            'gzclient'
        ],
        cwd=[package_dir],
        output='screen'
    )

    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot.spawn.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw_pose': yaw_pose,
        }.items()
    )

    publish_state_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'state.publisher.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(start_gzserver_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(start_gzclient_cmd)
    ld.add_action(publish_state_cmd)

    return ld
