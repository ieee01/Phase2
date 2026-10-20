from setuptools import setup
import os
from glob import glob

package_name = 'cstam_core'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='CSTAM TuniBot Software Team',
    maintainer_email='dev@cstam.tunibot.org',
    description='Core logic nodes for CSTAM delivery robot',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'delivery_task_manager = cstam_core.delivery_task_manager:main',
            'docking_controller = cstam_core.docking_controller:main',
            'battery_simulator = cstam_core.battery_simulator:main',
            'dynamic_obstacle_controller = cstam_core.dynamic_obstacle_controller:main',
        ],
    },
)
