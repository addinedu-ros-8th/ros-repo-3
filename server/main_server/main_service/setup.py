from setuptools import setup

package_name = 'main_service'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # 명시적으로 main_service만 포함
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'rclpy_action',
        # 'shared_interfaces',  # pip 설치 불가한 로컬 패키지는 제거
    ],
    zip_safe=True,
    maintainer='yonmilk',
    maintainer_email='yonmilk@icloud.com',
    description='Main service ROS2 interface package',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 메인 실행 노드
            'main = main_service.main:main',

            # 액션 클라이언트
            'start_delivery_client = main_service.action.start_delivery_client:main',
            'move_to_goal_client = main_service.action.move_to_goal_client:main',
            'scan_inventory_client = main_service.action.scan_inventory_action_client:main',
            'maintenance_charge_client = main_service.action.maintenance_charge_action_client:main',
            'security_patrol_client = main_service.action.security_patrol_action_client:main',

            # 서비스 서버
            'log_query_service = main_service.service.log_query_service:main',
            'manager_login_service = main_service.service.manager_login_service:main',

            # 퍼블리셔
            'log_event_publisher = main_service.publisher.log_event_publisher:main',
            'control_command_publisher = main_service.publisher.control_command_publisher:main',
            'pose_sender = main_service.publisher.pose_sender:main',

            # 구독자
            'sensor_listener_node = main_service.subscriber.sensor_listener_node:main',

            # Main → Mobile 퍼블리셔
            'charge_command_publisher = main_service.publisher.charge_command_publisher:main',
            'navigation_goal_publisher = main_service.publisher.navigation_goal_publisher:main',
            'dashboard_status_publisher = main_service.publisher.dashboard_status_publisher:main',
            'obstacle_response_publisher = main_service.publisher.obstacle_response_publisher:main',
            'avoidance_cmd_publisher = main_service.publisher.avoidance_cmd_publisher:main',
            'precision_stop_cmd_publisher = main_service.publisher.precision_stop_cmd_publisher:main',

            # Mobile → Main 구독자
            'task_progress_subscriber = main_service.subscriber.task_progress_subscriber:main',
            'task_complete_subscriber = main_service.subscriber.task_complete_subscriber:main',
            'precision_stop_result_subscriber = main_service.subscriber.precision_stop_result_subscriber:main',
            'obstacle_detected_subscriber = main_service.subscriber.obstacle_detected_subscriber:main',
            'battery_subscriber = main_service.subscriber.battery_subscriber:main',
        ],
    },
)
