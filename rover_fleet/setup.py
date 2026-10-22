from setuptools import find_packages, setup

package_name = 'rover_fleet'

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
    maintainer='Jerry',
    maintainer_email='ogjeche3342@gmail.com',
    description='Fleet nodes for Gazebo: radio/base-station emulator and per-robot agent.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'comms_sim = rover_fleet.comms_sim:main',
            'fleet_agent = rover_fleet.fleet_agent:main',
            'record_run = rover_fleet.record_run:main',
        ],
    },
)
