from glob import glob

from setuptools import find_packages, setup

package_name = 'core_helm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BlueLab-ICM',
    maintainer_email='bluelab@icm.csic.es',
    description='The single command path between modules and the vehicle.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'helm_node = core_helm.helm_node:main',
            'fleet_node = core_helm.fleet_node:main',
        ],
    },
)
