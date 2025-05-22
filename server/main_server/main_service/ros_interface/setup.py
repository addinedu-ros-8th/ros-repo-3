from setuptools import setup
import os

package_name = 'main_service'

# 루트 경로(루트: server/main_server/main_service 혹은 ros_interface)에 따라 경로 수정 필요
setup(
    name=package_name,
    version='0.0.1',
    packages=[
        'ros_interface',
        'ros_interface.action',
        'ros_interface.publisher',
        'ros_interface.service',
        'ros_interface.subscriber',
    ],
    package_dir={'ros_interface': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # config 파일 설치
        (os.path.join('share', package_name, 'config'), ['config/goal_position.json',
                                                         'config/nav2_params.yaml']),
        # launch 파일 설치
        (os.path.join('share', package_name, 'launch'), ['launch/planner_launch.launch.xml']),
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
            # 액션 클라이언트
            'start_delivery_client           = ros_interface.action.start_delivery_client:main',
            'move_to_goal_client             = ros_interface.action.move_to_goal_client:main',
            'scan_inventory_client           = ros_interface.action.scan_inventory_action_client:main',
            'maintenance_charge_client       = ros_interface.action.maintenance_charge_action_client:main',
            'security_patrol_client          = ros_interface.action.security_patrol_action_client:main',
            # 서비스·퍼블리셔·서브스크라이버
            'log_query_service               = ros_interface.service.log_query_service:main',
            'log_event_publisher             = ros_interface.publisher.log_event_publisher:main',
            'control_command_publisher       = ros_interface.publisher.control_command_publisher:main',
            'sensor_listener_node            = ros_interface.subscriber.sensor_listener_node:main',
            'pose_sender                     = ros_interface.publisher.pose_sender:main',
            # 토픽 퍼블리셔 (Main → Mobile)
            'charge_command_publisher        = ros_interface.publisher.charge_command_publisher:main',
            'navigation_goal_publisher       = ros_interface.publisher.navigation_goal_publisher:main',
            'dashboard_status_publisher      = ros_interface.publisher.dashboard_status_publisher:main',
            'obstacle_response_publisher     = ros_interface.publisher.obstacle_response_publisher:main',
            'avoidance_cmd_publisher         = ros_interface.publisher.avoidance_cmd_publisher:main',
            'precision_stop_cmd_publisher    = ros_interface.publisher.precision_stop_cmd_publisher:main',
            # 토픽 서브스크라이버 (Mobile → Main)
            'task_progress_subscriber        = ros_interface.subscriber.task_progress_subscriber:main',
            'task_complete_subscriber        = ros_interface.subscriber.task_complete_subscriber:main',
            'precision_stop_result_subscriber= ros_interface.subscriber.precision_stop_result_subscriber:main',
            'obstacle_detected_subscriber    = ros_interface.subscriber.obstacle_detected_subscriber:main',
            # 새로 추가된 플래너 노드
            'global_planner                  = ros_interface.global_planner:main',
            'path_follower                   = ros_interface.path_follower:main',
        ],
    },
)
