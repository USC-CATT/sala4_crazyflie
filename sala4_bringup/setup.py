from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'sala4_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gary',
    maintainer_email='pinales@usc.edu',
    description='TODO: Package description',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world = sala4_bringup.hello_world:main',
            'gz_odom_logger = sala4'
        ],
    },
)
