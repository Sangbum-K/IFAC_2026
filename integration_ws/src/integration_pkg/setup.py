from setuptools import setup
package_name = 'integration_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        (
            'share/' + package_name + '/config/environments',
            [
                'config/environments/sim.yaml',
                'config/environments/mapping.yaml',
                'config/environments/race_map.yaml',
                'config/environments/real_car.yaml',
            ],
        ),
        ('share/' + package_name + '/launch', ['launch/stack.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Integration package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'integration_node = integration_pkg.integration_node:main',
        ],
    },
)
