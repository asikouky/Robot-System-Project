from setuptools import find_packages, setup

package_name = 'mission_state'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lordboussougou',
    maintainer_email='junior-stevy-randy.boussougou@polymtl.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    ##tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_state = mission_state.mission_state:main',
        ],
    },
)
