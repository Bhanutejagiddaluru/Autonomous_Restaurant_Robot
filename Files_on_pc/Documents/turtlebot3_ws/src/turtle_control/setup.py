from setuptools import find_packages, setup

package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='res_user',
    maintainer_email='res_user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'turtle_control = turtle_control.turtle_control:main',
        'ir_sensor_subscriber = turtle_control.ir_sensor_subscriber:main',
        'emercency_publisher = turtle_control.emergency_publisher:main',
        ],
    },
)
