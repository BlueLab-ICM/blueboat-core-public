from setuptools import find_packages, setup

package_name = 'core_autonomy'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'psutil'],
    zip_safe=True,
    maintainer='BLUE Lab, ICM-CSIC',
    maintainer_email='bluelab@icm.csic.es',
    description=(
        'Gateway keepalive, safety-gated command relay, and monitoring for '
        'the BlueBoat backseat.'
    ),
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'brain_node = core_autonomy.brain_node:main',
            'sys_logger_node = core_autonomy.sys_logger_node:main',
            'telemetry_logger_node = core_autonomy.telemetry_logger_node:main',
        ],
    },
)
