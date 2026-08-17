from setuptools import find_packages, setup

package_name = 'core_gateway'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='BLUE Lab, ICM-CSIC',
    maintainer_email='bluelab@icm.csic.es',
    description=(
        'MAVLink gateway between the ROS 2 backseat and the ArduPilot '
        'frontseat. The only node authorised to command the vehicle.'
    ),
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'gateway_node = core_gateway.gateway_node:main',
        ],
    },
)
