from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        # Launch the patrol_node
        Node(
            package='robot_patrol',
            executable='direction_service_node',
            name='direction_service_node',
            output='screen',
            parameters=[],
        ),

    ])
