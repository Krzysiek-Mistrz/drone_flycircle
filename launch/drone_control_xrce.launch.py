import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500'],
        cwd=os.path.expanduser('~/PX4-Autopilot'),
        output='screen'
    )

    # Uruchom Micro-ROS Agent
    micro_ros_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # Node sterujący dronem
    drone_control_node = Node(
        package='drone_fly_circle',
        executable='drone_fly_circle',
        name='drone_fly_circle',
        output='screen'
    )

    return LaunchDescription([
        px4_sitl,
        micro_ros_agent,
        drone_control_node,
    ])
