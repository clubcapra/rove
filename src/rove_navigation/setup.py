from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rove_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rove',
    maintainer_email='capra@ens.etsmtl.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_to_person = rove_navigation.navigate_to_person:main',
            'green_person_tracker = rove_navigation.green_person_tracker:main',
            'behavior_square_pattroling = rove_navigation.behavior_square_pattroling:main',
            'person_following = rove_navigation.person_following:main',
            'frontier_publisher = rove_navigation.frontier_publisher:main',
            'exploration = rove_navigation.exploration:main',
            'lost_connection = rove_navigation.lost_connection:main',
        ],
    },
)
