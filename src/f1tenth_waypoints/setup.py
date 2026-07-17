from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_waypoints'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='wxxn1004@gmail.com',
    description='Offline centerline extraction + waypoint/global_path publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_loader = f1tenth_waypoints.waypoint_loader:main',
        ],
    },
)
