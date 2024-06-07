from setuptools import find_packages, setup

package_name = 'prototype'

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
    maintainer='codie',
    maintainer_email='codie@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_hello = prototype.test_node:main',
            'test_laser = prototype.test_avoidance:main',
            'test_angle = prototype.test_swarm:main',
            'start_menu = prototype.menu:main',
            'swarm = prototype.swarmbot:main'
        ],
    },
)
