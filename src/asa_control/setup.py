from setuptools import find_packages, setup

package_name = 'asa_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_startup.launch.py']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dennis',
    maintainer_email='unrecognizedmusic@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_key = asa_control.teleop_key:main',
            'serial_communicator = asa_control.serial_communicator:main',
            'serial_reader = asa_control.serial_reader:main'
        ],
    },
)
