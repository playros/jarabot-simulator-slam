from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jarabot_sim_cartographer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='playros',
    maintainer_email='you@example.com',
    description='Jarabot Cartographer SLAM bringup (launch + lua + rviz).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # bringup only -> no nodes here
        ],
    },
)
