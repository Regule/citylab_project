from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Path to the RViz config file
    rviz_config_file = os.path.join(
        get_package_share_directory('robot_patrol'),
        'rviz',
        'patrol_vizualization_real_robot.rviz'
    )


    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='direction_service_node',
            name='direction_service_node',
            output='screen',
            parameters=[],
        ),

        Node(
            package='robot_patrol',
            executable='patrol_with_service_node',
            name='patrol_with_service_node',
            output='screen',
            parameters=[],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
