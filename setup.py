from setuptools import setup
from glob import glob

package_name = 'echo_mcu'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bitbyte08',
    maintainer_email='me@bitworkspace.kr',
    description='MCU serial interface',
    license='MIT',
    tests_require=['pytest'],
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'cmd_node = echo_mcu.cmd_node:main',
            'cmd_node_v2 = echo_mcu.cmd_node_v2:main',
            'odom_node = echo_mcu.odom_node:main',
            'serial_node = echo_mcu.serial_node:main',
            'dummy_serial_node = echo_mcu.dummy_serial_node:main',
            'tf_node=echo_mcu.tf_node:main'
        ],
    },
)
