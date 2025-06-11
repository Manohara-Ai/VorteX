from glob import glob
import os
from setuptools import setup

package_name = 'vortex'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ] + [
        (os.path.join('share', package_name, os.path.dirname(f)), [f])
        for f in glob('models/**/*', recursive=True)
        if os.path.isfile(f)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manohara',
    maintainer_email='manohara01012005@gmail.com',
    description='Sim package',
    license='MIT',
)
