from setuptools import find_packages, setup

package_name = 'ros_gz_example_application'

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
    maintainer='zineb',
    maintainer_email='zbenzeroual@gmail.com',
    description='ROS 2 package for the identifier command',
    license='Apache-2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'identifier_command = ros_gz_example_application.identifier_command:main',
            'random_navigator = ros_gz_example_application.random_navigator:main',
        ],
    },
)
