from setuptools import setup
import os
from glob import glob

package_name = 'swarm'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name),glob('launch/*.launch.py')),
        (os.path.join('share',package_name),glob('launch/*.py')),
        (os.path.join('share',package_name),glob('models/turtlebot3_burger/*.sdf')),
#        (os.path.join('share',package_name),glob('rviz/*')),
#        (os.path.join('share',package_name),glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swadhin',
    maintainer_email='swadhin12a@gmail.com',
    description='tb3 swarm',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm = swarm.pubsub:pubsub',
            'spawn_tb3 = swarm.spawn_tb3:main'
        ],
    },
)
