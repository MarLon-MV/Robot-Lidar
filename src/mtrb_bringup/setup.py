from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'mtrb_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Brandon Vicente',
    maintainer_email='brandonvicente716@gmail.com',
    description='Bringup for the MTRB robot (Create 3 + 2D LiDAR, Nav2)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'initial_pose_publisher = mtrb_bringup.initial_pose_publisher:main',
        ],
    },
)
