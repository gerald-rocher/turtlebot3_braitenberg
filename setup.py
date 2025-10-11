from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot3_braitenberg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='humble',
    maintainer_email='humble@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'vehicle2a_node = turtlebot3_braitenberg.vehicle2a_node:main',
           'vehicle2b_node = turtlebot3_braitenberg.vehicle2b_node:main',           
        ],
    },
)
