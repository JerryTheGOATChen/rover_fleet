from setuptools import find_packages, setup

package_name = 'fleet_rl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['models/*.npz', 'models/*.json']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Jerry',
    maintainer_email='ogjeche3342@gmail.com',
    description='Multi-robot exploration under limited communication (simulator + RL).',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
)
