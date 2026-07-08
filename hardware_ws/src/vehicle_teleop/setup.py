from setuptools import setup

package_name = 'vehicle_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Joystick teleoperation nodes for the F1TENTH vehicle',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ackermann_joy_teleop = vehicle_teleop.ackermann_joy_teleop:main',
        ],
    },
)
