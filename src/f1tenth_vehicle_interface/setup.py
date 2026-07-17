from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_vehicle_interface'

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
    description='Sim/real vehicle swap point for /drive commands',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_vehicle_node = f1tenth_vehicle_interface.sim_vehicle_node:main',
            'real_vehicle_node = f1tenth_vehicle_interface.real_vehicle_node:main',
        ],
    },
)
