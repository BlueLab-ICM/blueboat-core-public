from glob import glob

from setuptools import find_packages, setup

package_name = 'core_ops'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'psutil'],
    zip_safe=True,
    maintainer='BlueLab-ICM',
    maintainer_email='bluelab@icm.csic.es',
    description='Mission recording and vehicle health: events, telemetry, host monitoring.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'recorder_node = core_ops.recorder_node:main',
            'telemetry_node = core_ops.telemetry_node:main',
            'monitor_node = core_ops.monitor_node:main',
        ],
    },
)
