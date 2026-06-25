from setuptools import setup

package_name = 'vehicle_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/config',
            [
                'config/params.yaml',
            ],
        ),
        (
            'share/' + package_name + '/config/environments',
            [
                'config/environments/sim.yaml',
                'config/environments/mapping.yaml',
                'config/environments/race_map.yaml',
                'config/environments/real_car.yaml',
            ],
        ),
        (
            'share/' + package_name + '/launch',
            [
                'launch/hardware.launch.py',
            ],
        ),
        ('share/' + package_name + '/urdf', ['urdf/f1tenth_vehicle.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Hardware bringup package for F1TENTH vehicle',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'estop_node = vehicle_bringup.estop_node:main',
        ],
    },
)
