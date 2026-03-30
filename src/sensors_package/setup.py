from setuptools import find_packages, setup

package_name = 'sensors_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sumoth',
    maintainer_email='tech@agirsailing.se',
    description='ROS2 GPS package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'talker_gps = sensors_package.gps_node:main',
		    'listener_gps = sensors_package.gps_listener:main',
            'talker_ultrasonic = sensors_package.ultrasonic_node:main',
            'talker_imu = sensors_package.imu_node:main'
	],
    },
)
