from setuptools import setup

package_name = 'localization_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/config/localization',
            [
                'config/localization/slam_toolbox_mapping.yaml',
                'config/localization/slam_toolbox_localization.yaml',
                'config/localization/amcl.yaml',
                'config/localization/map_server.yaml',
            ],
        ),
        (
            'share/' + package_name + '/launch',
            [
                'launch/localization.launch.py',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Localization bringup package for F1TENTH vehicle',
    license='Apache-2.0',
)
