from setuptools import find_packages, setup
from glob import glob


package_name = 'sobit_light_teleop'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Keith Valentin',
    maintainer_email='kvalentincardenas@hotmail.com',
    description='SOBIT LIGHT teleoperation systems',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'keyboard_nav_teleop = sobit_light_teleop.keyboard_nav_teleop:main',
            'keyboard_arm_teleop = sobit_light_teleop.keyboard_arm_teleop:main',
            'keyboard_all_teleop = sobit_light_teleop.keyboard_all_teleop:main',
            'vr_teleop_1 = sobit_light_teleop.vr_teleop_1:main',
            'vr_teleop_2 = sobit_light_teleop.vr_teleop_2:main',
            'dualshock_teleop = sobit_light_teleop.dualshock_teleop:main'
        ],
    },
)
