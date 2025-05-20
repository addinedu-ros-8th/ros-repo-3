from setuptools import setup, find_packages

package_name = 'main_service'
package_path = 'server/main_server/main_service/ros_interface'
package_path_relative = package_path.replace('/', '.')

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[f'{package_path_relative}.*'],
                           where='package_path_relative'),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yonmilk',
    maintainer_email='yonmilk@icloud.com',
    description='Main service ROS2 interface package',
    license='MIT',
    entry_points={
        'console_scripts': [
            f'start_delivery_client = {package_path_relative}.action.start_delivery_client:main',
            f'log_query_service = {package_path_relative}.service.log_query_service:main',
            f'log_event_publisher = {package_path_relative}.publisher.log_event_publisher:main',
            f'control_command_publisher = {package_path_relative}.publisher.control_command_publisher:main',
            f'sensor_listener_node = {package_path_relative}.subscriber.sensor_listener_node:main',
        ],
    },
)
