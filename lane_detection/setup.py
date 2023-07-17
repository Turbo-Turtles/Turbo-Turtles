from setuptools import setup

package_name = 'lane_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='georg',
    maintainer_email='georg@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = " + package_name + "temp_nodes:main",
            "lane_det_1 = " + package_name + "lane_det_1:main",
            "lane_det_2 = " + package_name + "lane_det_2:main",
            "lane_det_3 = " + package_name + "lane_det_3:main",
            "test = " + package_name + "test:main",
            "lane = " + package_name + ".lane_complete_v1:main",
        ],
    },
)
