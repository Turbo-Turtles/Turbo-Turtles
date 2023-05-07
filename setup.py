import os
from glob import glob
from setuptools import setup

package_name = 'lidar_func'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name + '/launch', ['launch/', '*launch.[pxy][yma]*']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.lua*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robert',
    maintainer_email='robert@mi-s.de',
    description='Disposable test environment for everything related to the LiDAR',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occ_grid = lidar_func.occupancy_grid_node:main',
            'stick_to_wall = lidar_func.stick_to_wall:main',
            'mapping = lidar_func.mapping:main',
            'go_to_pose = lidar_func.simple_commander_api:main',
        ],
    },
)
