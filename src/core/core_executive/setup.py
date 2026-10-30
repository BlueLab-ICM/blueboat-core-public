from setuptools import find_packages, setup

package_name = 'core_executive'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BlueLab-ICM',
    maintainer_email='bluelab@icm.csic.es',
    description='Mission executive: run a behaviour tree and own the run lifecycle.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'executive_node = core_executive.executive_node:main',
            'executive_cli = core_executive.cli:main',
        ],
    },
)
