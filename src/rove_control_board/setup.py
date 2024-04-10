import io
import os
import sys
from setuptools import find_packages, setup

package_name = 'rove_control_board'

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
    maintainer='capra',
    maintainer_email='capra@ens.etsmtl.ca',
    description='Implements communication with the control board through ROS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = rove_control_board.control_board_bridge:main',
        ],
    },
)

from rove_control_board.api import manager


API_DIR = os.path.curdir + '/api'
API_FILE = API_DIR + '/api.h'
mode = 'x'
if not os.path.exists(API_DIR):
    os.mkdir(API_DIR)
if os.path.exists(API_FILE):
    mode = 'w'

with io.open(API_FILE, mode) as wr:
    wr.write(manager.buildAPI())