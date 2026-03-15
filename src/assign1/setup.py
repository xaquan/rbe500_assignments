from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'assign1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Install gazebo config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        # Install URDF/Xacro
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),

        # Install RViz configs (optional but recommended)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),

        # Install world files (optional, only needed if you have custom worlds or want to include the empty world)
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='quan.h.xa@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fw_solver = assign1.fw_solver:main',
            'inv_service = assign1.inv_service:main',
        ],
    },
)
