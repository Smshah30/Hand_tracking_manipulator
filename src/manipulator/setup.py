from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'manipulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'support_launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wad',
    maintainer_email='wad@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            'check_arm = manipulator.basic_control:main',
            'pd_controller = manipulator.pd_controller:main',
            'fkine = manipulator.fkine:main',
            'invkine = manipulator.invkine:main',
            'connector_arm_hand = manipulator.connector_arm_hand:main'

            
        ],
    },
)
