from setuptools import setup, find_packages
import os

package_name = 'main_service'

# Treat 'server/main_server/main_service' as the root of the Python package
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='server/main_server/main_service'),
    package_dir={'': 'server/main_server/main_service'},
    data_files=[
        # ament resource index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package manifest
        ('share/' + package_name, ['package.xml']),
        # configuration files
        ('share/' + package_name + '/config', ['ros_interface/config/goal_position.json']),
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
            # main entry point
            'main = main_service.main:main',

            # action clients
            'maintenance_charge_action_client = main_service.ros_interface.action.maintenance_charge_action_client:main',
            'move_to_goal_client            = main_service.ros_interface.action.move_to_goal_client:main',
            'scan_inventory_action_client  = main_service.ros_interface.action.scan_inventory_action_client:main',
            'security_patrol_action_client = main_service.ros_interface.action.security_patrol_action_client:main',
            'start_delivery_client         = main_service.ros_interface.action.start_delivery_client:main',

            # topic publishers (Mobile Controller)
            'avoidance_cmd_publisher      = main_service.ros_interface.publisher.avoidance_cmd_publisher:main',
            'charge_command_publisher     = main_service.ros_interface.publisher.charge_command_publisher:main',
            'control_command_publisher    = main_service.ros_interface.publisher.control_command_publisher:main',
            'dashboard_status_publisher   = main_service.ros_interface.publisher.dashboard_status_publisher:main',
            'log_event_publisher          = main_service.ros_interface.publisher.log_event_publisher:main',
            'navigation_goal_publisher    = main_service.ros_interface.publisher.navigation_goal_publisher:main',
            'obstacle_response_publisher  = main_service.ros_interface.publisher.obstacle_response_publisher:main',
            'precision_stop_cmd_publisher = main_service.ros_interface.publisher.precision_stop_cmd_publisher:main',

            # topic publisher (Manager)
            'pose_sender = main_service.ros_interface.publisher.pose_sender:main',

            # topic subscribers (Mobile Controller)
            'obstacle_detected_subscriber       = main_service.ros_interface.subscriber.obstacle_detected_subscriber:main',
            'precision_stop_result_subscriber   = main_service.ros_interface.subscriber.precision_stop_result_subscriber:main',
            'sensor_listener_node              = main_service.ros_interface.subscriber.sensor_listener_node:main',
            'task_complete_subscriber          = main_service.ros_interface.subscriber.task_complete_subscriber:main',
            'task_progress_subscriber          = main_service.ros_interface.subscriber.task_progress_subscriber:main',

            # services (Manager)
            'log_query_service    = main_service.ros_interface.service.log_query_service:main',
            'manager_login_service = main_service.ros_interface.service.manager_login_service:main',
        ],
    },
)
