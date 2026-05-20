from setuptools import find_packages, setup

package_name = 'alphabot2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'config/robot_controller.yaml', 'config/gamepad.yaml', 'config/mapper_params.yaml', 'config/nav2_params.yaml', 'maps/kancel_map.yaml', 'maps/kancel_map.pgm', 'maps/kancel_map-big.yaml', 'maps/kancel_map-big.pgm']),
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
            'alphabot2_node = alphabot2.alphabot2_node:main'
        ],
    },
)
