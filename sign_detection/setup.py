from setuptools import setup

package_name = 'sign_detection'

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
    maintainer='marvin',
    maintainer_email='marvin.menzel02@gmail.com',
    description='sign detection for turtlebot3',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sign_detection_node = ' + package_name + '.sign_detection:main',
            'manual_detection_node = ' + package_name + '.manual_signs:main', 
        ],
    },
)
