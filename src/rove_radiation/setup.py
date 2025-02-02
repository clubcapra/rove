from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rove_radiation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='janice',
    maintainer_email='janice@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radiation_publisher = rove_radiation.radiation_publisher:main',
            'radiation_position_tracker = rove_radiation.radiation_position_tracker:main',
        ],
    },
)
