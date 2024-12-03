from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simulations'
robo_package = 'robot_description'
world_package = 'worlds'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install all URDF files from robot_description
        (os.path.join('share', robo_package, 'urdf'), glob('robot_description/urdf/*')),
        # Install all world files from the worlds package
        (os.path.join('share', world_package, 'worlds'), glob('worlds/*.world'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naveenkumar',
    maintainer_email='naveenmooram111@gmail.com',
    description='A ROS 2 package for simulating robots in Gazebo',
    license='Apache 2.0',  # Replace with the actual license if different
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # You can add your Python executable scripts here in the future
        ],
    },
)
