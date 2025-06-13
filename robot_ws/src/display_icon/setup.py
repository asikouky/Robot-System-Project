from setuptools import setup

package_name = 'display_icon'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'pygame'],
    zip_safe=True,
    maintainer='ton-nom',
    maintainer_email='ton@mail.com',
    description='Affiche une icône si le robot est le plus éloigné',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/display_icon']),
        ('share/display_icon', ['package.xml']),
        ('share/display_icon/launch', ['launch/display_launch.py']),
    ],
    entry_points={
        'console_scripts': [
            'display_farthest_node = display_icon.display_farthest_node:main',
        ],
    },
)
