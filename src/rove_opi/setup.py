import io
from setuptools import find_packages, setup

package_name = 'rove_opi'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name, *submodules],
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jocobc',
    maintainer_email='jocobc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = rove_opi.main:main'
        ],
    },
)
