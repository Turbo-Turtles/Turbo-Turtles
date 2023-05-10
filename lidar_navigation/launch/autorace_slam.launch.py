import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('lidar_navigation'),'config')
    map_file = os.path.join(config_dir, 'tb3_map.yaml')
    param_file = os.path.join(config_dir, 'burger_params.yaml')
    rviz_file = os.path.join(config_dir, 'navigation.rviz')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('turtlebot3_gazebo'), '/launch', '/turtlebot3_autorace_2020.launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch', '/bringup_launch.py']),
            launch_arguments={
                'slam':'True',
                'map':map_file,
                'use_sim_time':'True',
                'params_file':param_file,
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            output='screen',
            arguments=[
                '-d', rviz_file,
            ]
        )
    ])