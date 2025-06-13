from setuptools import setup

package_name = 'limo_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TonNom',
    maintainer_email='TonEmail@example.com',
    description='A ROS 2 package to move AgileX LIMO forward',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_limo = limo_controller.move_limo:main',
        ],
    },
)
