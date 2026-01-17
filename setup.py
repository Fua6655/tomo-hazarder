from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'control_tomo'

setup(
    name=package_name,
    version='1.3.1',
    packages=find_packages(include=['control_tomo', 'control_tomo.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joy_serial.launch.py']),
        ('share/' + package_name + '/launch', ['launch/joy_udp.launch.py']),
        ('share/' + package_name + '/launch', ['launch/turtlesim.launch.py']),
        ('share/' + package_name + '/web/html', glob('control_tomo/web/html/index.html')),
        ('share/' + package_name + '/web/html/css', glob('control_tomo/web/html/css/*.css')),
        ('share/' + package_name + '/web/html/js', glob('control_tomo/web/html/js/*.js')),
    ],
    install_requires=[
        'setuptools'
    ],
    package_data={
        'control_tomo': [
            'web/html/*',
            'web/html/css/*',
            'web/html/js/*',
            'web/html/fonts/*',
        ]
    },
    #include_package_data=True,
    #zip_safe=True,
    maintainer='luka',
    maintainer_email='luka.fua2@gmail.com',
    description='Control Tomo Vinkovic',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'ps4_teleop = control_tomo.ps4_teleop_node:main',
        'arduino_serial = control_tomo.arduino_serial_node:main',
        'esp_udp = control_tomo.esp_udp_node:main',
        'control_factory = control_tomo.control_factory_node:main',
        'web_server = control_tomo.web.web_server:main',
        ],
    },
)
