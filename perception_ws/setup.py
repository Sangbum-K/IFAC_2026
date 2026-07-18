import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'f1tenth_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='f1tenth_team',
    maintainer_email='team@example.com',
    description='F1TENTH 장애물 인지: 클러스터링 + Kalman Filter 트래킹',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector_node = f1tenth_perception.obstacle_detector_node:main',
        ],
    },
)
