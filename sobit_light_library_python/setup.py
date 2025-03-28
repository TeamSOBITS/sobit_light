import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sobit_light_library_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sobits',
    maintainer_email='kvalentincardenas@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'sobit_light_wheel_controller = sobit_light_library_python.sobit_light_wheel_controller:main',
            'sobit_light_joint_controller = sobit_light_library_python.sobit_light_joint_controller:main',
            'test_control_arm = sobit_light_library_python.test_control_arm:main'
        ],
    },
)
