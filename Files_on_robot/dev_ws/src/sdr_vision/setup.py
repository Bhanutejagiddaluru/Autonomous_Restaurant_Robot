from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sdr_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/exported_model', glob('exported_model/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cricel',
    maintainer_email='cricel.design@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect = sdr_vision.detect:main',
            'screenshot = sdr_vision.screenshot:main',
            'camera_to_ros = sdr_vision.camera_to_ros:main'
        ],
    },
)
