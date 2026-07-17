from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='wxxn1004@gmail.com',
    description='F1TENTH capstone integration bringup (sim/real switch)',
    license='MIT',
    tests_require=['pytest'],
)
