from glob import glob
import os
from setuptools import setup, find_packages

package_name = 'vortex'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['vortex', 'vortex.*', 'depth_anything_v2', 'depth_anything_v2.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/checkpoints', glob('checkpoints/*.pth')),
    ] + [
        (os.path.join('share', package_name, os.path.dirname(f)), [f])
        for f in glob('models/**/*', recursive=True)
        if os.path.isfile(f)
    ],
    install_requires=[
        'setuptools',
        'torch',
        'opencv-python',
        'numpy<=2.0',
        
    ],
    zip_safe=True,
    maintainer='manohara',
    maintainer_email='manohara01012005@gmail.com',
    description='Sim package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = vortex.perception.vision_node:main',
            'lidar_node = vortex.mapping.lidar_node:main',
            'planner_node = vortex.planning.planner_node:main',
        ],
    },
)
