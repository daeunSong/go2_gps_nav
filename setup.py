from setuptools import setup

package_name = 'go2_gps_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daeun Song',
    maintainer_email='daeun7250@gmail.com',
    description='Minimal GPS navigation for ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gps_nav = go2_gps_nav.gps_navigation:main',
        ],
    },
)