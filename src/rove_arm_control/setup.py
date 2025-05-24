from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'rove_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rove',
    maintainer_email='capra@ens.etsmtl.ca',
    description='Package to control the rove kinova robotic arm using a 3d mouse',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rove_arm_control = rove_arm_control.arm_control:main'
        ],
    },
)
