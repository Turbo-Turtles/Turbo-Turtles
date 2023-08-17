import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='autorace_core',
            executable='intelligence',
            name='intelligence',
            output='screen',
        ),
        
        Node(
            package='lidar_navigation',
            executable='tunnel_mission',
            name='tunnel_mission',
            output='screen',
        ),

        Node(
            package='lidar_navigation',
            executable='construction_mission',
            name='construction_mission',
            output='screen',
        ),

        Node(
            package='lane_detection',
            executable='lane',
            name='lane_complete_v1',
            output='screen',
        ),

        Node(
            package='sign_detection',
            executable='manual_detection_node',
            name='manual_signs',
            output='screen',
        ),
    ])