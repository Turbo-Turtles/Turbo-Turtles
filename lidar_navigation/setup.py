import os
from glob import glob
from setuptools import setup

package_name = 'lidar_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name + '/launch', ['launch/', '*launch.py']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robert Schneider',
    maintainer_email='robert@mi-s.de',
    description='Reads inputs from LiDAR scanner and uses this information to navigate.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'single_goal_nav = ' + package_name + '.single_goal_nav:main',
            'get_robot_pos = ' + package_name + '.get_robot_position:main',
            'drive_1m = ' + package_name + '.drive_1m:main',
            'waypoint_follower = ' + package_name + '.waypoint_follower:main',
            'waypoint_nav = ' + package_name + '.waypoint_nav:main',
            'tunnel_mission_test01 = ' + package_name + '.tunnel_mission_test01:main',
        ],
    },
)