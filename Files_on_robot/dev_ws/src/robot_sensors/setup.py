from setuptools import setup

package_name = 'robot_sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ksu-robotics',
    maintainer_email='ksu-robotics@todo.todo',
    description='ROS 2 package for Raspberry Pi IR sensor reading',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ir_sensor_publisher = robot_sensors.ir_sensor_publisher:main',
        ],
    },
)
