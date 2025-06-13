from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'identifier_command'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ajout des fichiers de lancement
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lordboussougou',
    maintainer_email='junior-stevy-randy.boussougou@polymtl.ca',
    description='ROS2 Identifier Command Service',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'identifier_command = identifier_command.identifier_command:main',
        ],
    },
)
