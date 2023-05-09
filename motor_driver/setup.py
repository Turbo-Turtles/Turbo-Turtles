from setuptools import setup

package_name = 'motor_driver'

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
    maintainer='robert',
    maintainer_email='robert@mi-s.de',
    description='converts simple inputs into output format for the robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver_node = motor_driver.motor_driver_node:main',
        ],
    },
)
