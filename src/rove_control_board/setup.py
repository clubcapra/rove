import io
import os
import sys
import pip
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
    install_requires=[
        'setuptools',
        'capra_micro_comm_py @ git+https://github.com/clubcapra/capra_micro_comm_py.git@master', # THIS ISN'T WORKING...
        ],
    # requires=['capra_micro_comm_py @ git+https://github.com/clubcapra/capra_micro_comm_py.git@master'],
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
# PLEASE HELP!!!
# TODO Fix this please, I am losing my mind, I have been trying for hours to make this setup script install the python package from git...
# This works but I hate it and I want it to burn and die...
# i dont like it
try:
    import capra_micro_comm_py
except ImportError:
    pip.main(['install', 'capra_micro_comm_py @ git+https://github.com/clubcapra/capra_micro_comm_py.git@master', '--no-input', '--quiet'])

# FYI, to remove this package, run `pip uninstall capra_micro_comm_py`


# Generate the micro comm api for the control board
from rove_control_board.api import manager

API_DIR = os.path.curdir + '/api'
API_FILE = API_DIR + '/api.h'
mode = 'x'
if not os.path.exists(API_DIR):
    os.mkdir(API_DIR)
if os.path.exists(API_FILE):
    mode = 'w'

with io.open(API_FILE, mode) as wr:
    wr.write(manager.generateAPI())