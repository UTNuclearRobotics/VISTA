from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vista_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Dynamically include all launch files in the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='talal',
    maintainer_email='talal.alotaibi@utexas.edu',
    description='Simulation package for Dubins-based vehicle path planning and execution.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dubins_pose_to_pose_action_server = vista_sim.dubins_pose_to_pose_action_server:main',
            'dubins_pose_to_pose_action_client = vista_sim.dubins_pose_to_pose_action_client:main',
        ],
    },
)
