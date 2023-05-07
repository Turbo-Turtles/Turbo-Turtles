import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('lidar_func'),'config')
    map_file = os.path.join(config_dir, 'tb3_map.yaml')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot3_gazebo'),
                '/launch',
                '/turtlebot3_world.launch.py'
            ])
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            output='screen'
        )
    ])