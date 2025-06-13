from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, LogInfo, Shutdown
from launch.event_handlers import OnShutdown

def generate_launch_description():
    vision_node = Node(
        package='vortex',
        executable='vision_node',
        name='vision_node',
        namespace='perception',
        output='screen'
    )

    lidar_node = Node(
        package='vortex',
        executable='lidar_node',
        name='lidar_node',
        namespace='mapping',
        output='screen'
    )

    planner_node = Node(
        package='vortex',
        executable='planner_node',
        name='planner_node',
        namespace='planning',
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg='[launch] Starting all vortex nodes...'),

        vision_node,
        lidar_node,
        planner_node,
        
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    LogInfo(msg='Vortex system received Ctrl+C, shutting down all nodes...')
                ]
            )
        ),
    ])
