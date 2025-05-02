from setuptools import find_packages, setup

package_name = 'sensor_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'shared_interfaces'],
    zip_safe=True,
    maintainer='sang',
    maintainer_email='hinoonyaso@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_reader = sensor_reader.lidar_reader:main',
            'battery_reader = sensor_reader.battery_reader:main',
            'imu_reader = sensor_reader.imu_reader:main',
        ],
    },
)
