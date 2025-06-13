from setuptools import setup
import os
from glob import glob

package_name = 'robot_explorer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('robot_explorer/launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='asikouky',
    author_email='asikouky@gmail.com',
    maintainer='asikouky',
    maintainer_email='asikouky@gmail.com',
    keywords=['ROS2', 'exploration', 'rviz'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='Package pour l\'exploration du robot avec RViz',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'random_walk_node = robot_explorer.random_walk_node:main'
        ],
    },
)
