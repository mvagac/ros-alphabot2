import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'alphabot2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        #(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        ('share/' + package_name, ['package.xml', 'config/robot_controller.yaml', 'config/gamepad.yaml', 'config/mapper_params.yaml', 'config/nav2_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_benchmark_node = alphabot2.controller_benchmark_node:main',
            'nav2_benchmark_node = alphabot2.nav2_benchmark_node:main'
        ],
    },
)
