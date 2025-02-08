from setuptools import find_packages, setup

package_name = 'vortex'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/robot_visualize.config.rviz']),
        ('share/' + package_name + '/launch', ['launch/robot_world.launch.py', 'launch/robot_state_publisher.launch.py', 'launch/spawn_robot.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/sample_world.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manohara',
    maintainer_email='manohara01012005@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'vision_node = vortex.vision_node:main',
            'motion_node = vortex.motion_controller:main',
        ],
    },
)
