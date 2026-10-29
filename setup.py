from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'depth'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*')),

        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),

        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*')),
            
        (os.path.join('share', package_name, 'worlds'),
        glob('worlds/*')),
        
        (os.path.join('share', package_name, 'config'),
        glob('config/*.yaml') + glob('config/*.pgm')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soham',
    maintainer_email='soham@todo.todo',
    description='RTAB-Map depth-camera mapping, Nav2 navigation and frontier exploration for a rover in Gazebo and a real RealSense D435i',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'frontier_explorer = depth.frontier_explorer:main',
            'walk_gait = depth.walk_gait:main',
            'cal_servo = depth.cal_servo:main',
            'esp = depth.esp:main',
            'ground_truth_odom = depth.ground_truth_odom:main',
        ],
    },
)
