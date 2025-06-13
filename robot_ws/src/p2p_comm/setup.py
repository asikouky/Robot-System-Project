from setuptools import setup

package_name = 'p2p_comm'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TonNom',
    maintainer_email='TonEmail@example.com',
    description='P2P communication for robot distance sharing',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'p2p_node = p2p_comm.p2p_node:main',
        ],
    },
)
