from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'perception_pkg'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), ['config/params.yaml']),
        (
            os.path.join('share', package_name, 'config', 'environments'),
            glob('config/environments/*.yaml'),
        ),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='seok',
    maintainer_email='you@example.com',
    description='Laser obstacle detection and tracking pipeline for F1TENTH.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector_node = perception_pkg.obstacle_detector_node:main',
        ],
    },
)
