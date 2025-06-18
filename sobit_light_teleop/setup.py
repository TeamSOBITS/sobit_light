from setuptools import find_packages, setup

package_name = 'sobit_light_teleop'

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
    maintainer='sobits',
    maintainer_email='kvalentincardenas@hotmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyboard_nav_teleop = sobit_light_teleop.keyboard_nav_teleop:main',
            'keyboard_arm_teleop = sobit_light_teleop.keyboard_arm_teleop:main',
            'keyboard_all_teleop = sobit_light_teleop.keyboard_all_teleop:main'
        ],
    },
)
