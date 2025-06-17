from setuptools import setup
import os
from glob import glob

package_name = 'gtec_ftm'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Valentin Barral',
    maintainer_email='valentin.barral@udc.es',
    description='The gtec_ftm package - ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ESP32S2FTMTagReader = gtec_ftm.ESP32S2FTMTagReader:main',
            'ESP32S2FTMTagReaderExtra = gtec_ftm.ESP32S2FTMTagReaderExtra:main',
        ],
    },
) 