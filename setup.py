from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'doall'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'model'), glob('model/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        (os.path.join('share', package_name, 'arduino'), glob('arduino/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghaith-mhamdi',
    maintainer_email='ghaith-mhamdi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arduino_controller = doall.arduino_controller:main',
            'calibrate_joints_pub = doall.calibrate_joints_pub:main',
            'simple_action_server = doall.fibonacci:main',
        ],
    },
)
