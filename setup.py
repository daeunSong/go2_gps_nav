from setuptools import setup
import os
from glob import glob

package_name = 'go2_gps_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daeun Song',
    maintainer_email='daeun7250@gmail.com',
    description='GPS-based navigation with ROS2',
    license='Apache License 2.0',
    python_requires='>=3.8',
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'pyproj',
    ],
    entry_points={
        'console_scripts': [
            'gps_navigation_node = go2_gps_nav.gps_navigation_node:main',
        ],
    },
)