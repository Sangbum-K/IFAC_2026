from setuptools import setup

package_name = 'f1tenth_stack'

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
                'config/environments/mapping.yaml',
                'config/environments/race_map.yaml',
                'config/environments/real_car.yaml',
                'config/environments/sim.yaml',
            ],
        ),
        (
            'share/' + package_name + '/launch',
            [
                'launch/bringup.launch.py',
                'launch/bringup_launch.py',
                'launch/mapping.launch.py',
                'launch/teleop.launch.py',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='F1TENTH-style bringup package for vehicle launch and configuration',
    license='Apache-2.0',
)
