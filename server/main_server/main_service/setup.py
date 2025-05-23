from setuptools import setup, find_packages
import os

package_name = 'main_service'
base_path = 'server.main_server.main_service.ros_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[f'{base_path}*']),
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/main_service/config', ['ros_interface/config/goal_position.json']),
        (os.path.join('lib', package_name), ['main.py']),  
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'rclpy_action',
        'shared_interfaces',
    ],
    zip_safe=True,
    maintainer='yonmilk',
    maintainer_email='yonmilk@icloud.com',
    description='Main service ROS2 interface package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main = server.main_server.main_service.main:main',

            # action client
            *[
                f'{name} = {base_path}.action.{name}:main' for name in [
                    'maintenance_charge_action_client',
                    'move_to_goal_client',
                    'scan_inventory_action_client',
                    'security_patrol_action_client',
                    'start_delivery_client'
                ]
            ],

            # topic publisher (Mobile Controller)
            *[
                f'{name} = {base_path}.publisher.{name}:main' for name in [
                    'avoidance_cmd_publisher',
                    'charge_command_publisher',
                    'control_command_publisher',
                    'dashboard_status_publisher',
                    'log_event_publisher',
                    'navigation_goal_publisher',
                    'obstacle_response_publisher',
                    'precision_stop_cmd_publisher'
                ]
            ],

            # topic publisher (Manager)
            'pose_sender = ' + base_path + '.publisher.pose_sender:main',

            # topic subscriber (Mobile Controller)
            *[
                f'{name} = {base_path}.subscriber.{name}:main' for name in [
                    'obstacle_detected_subscriber',
                    'precision_stop_result_subscriber',
                    'sensor_listener_node',
                    'task_complete_subscriber',
                    'task_progress_subscriber'
                ]
            ],

            # service (Manager)
            *[
                f'{name} = {base_path}.service.{name}:main' for name in [
                    'log_query_service',
                    'manager_login_service'
                ]
            ],
        ],
    },
)
