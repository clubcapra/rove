from glob import glob
import io
import os
import pip
from setuptools import find_packages, setup
import sys
sys.path.append(__file__.removesuffix(f"/{__file__.split('/')[-1]}"))

package_name = 'rove_control_board'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[ # ROS2 does not follow the python setup procedure. Dependencies added here won't be installed
        'setuptools',
        'capra_micro_comm_py@git+https://github.com/clubcapra/capra_micro_comm_py.git@master', 
        ],
    # requires=['capra_micro_comm_py@git+https://github.com/clubcapra/capra_micro_comm_py.git@master'],
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
# - Iliana
try:
    import capra_micro_comm_py
except ImportError:
    # try:
    #     import can
    #     pip.main(['uninstall', 'python-can', '--no-input', '-y', '--quiet'])
    # except ImportError:
    #     pass
    with os.popen('wget https://github.com/clubcapra/capra_micro_comm_py/raw/refs/heads/master/dist/capra_micro_comm_py-1.2.1-py3-none-any.whl -q', 'w'):
        pass
    with os.popen('pip install ./capra_micro_comm_py-1.2.1-py3-none-any.whl --no-input --quiet', 'w'):
        pass
    with os.popen('rm -f ./capra_micro_comm_py-1.2.1-py3-none-any.whl'):
        pass

# try:
#     import can
# except ImportError:
#     pip.main(['install', 'python-can@git+https://github.com/IliTheButterfly/python-can.git@main', '--no-input', '--quiet'])

# FYI, to remove this package, run `pip uninstall capra_micro_comm_py python-can`


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
    api, _ = manager.generateAPI()
    wr.write(api)