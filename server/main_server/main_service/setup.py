from setuptools import setup, find_packages
import os

package_name = 'main_service'
base_path = 'server.main_server.main_service.ros_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='.', include=['server', 'server.*']),
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/main_service/config', ['ros_interface/config/goal_position.json']),
        ('share/main_service/config', ['ros_interface/config/nav2_params.yaml']),
        (os.path.join('share', package_name, 'launch'), ['ros_interface/launch/planner_launch.launch.xml']),
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
                    'start_delivery_client'
                ]
            ],

            # topic publisher (Mobile Controller)
            *[
                f'{name} = {base_path}.publisher.{name}:main' for name in [
                    'avoidance_cmd_publisher',

                    'log_event_publisher',
                    'obstacle_response_publisher',
                ]
            ],

            # topic publisher (Manager)
            'pose_sender = ' + base_path + '.publisher.pose_sender:main',

            # topic subscriber (Mobile Controller)
            *[
                f'{name} = {base_path}.subscriber.{name}:main' for name in [
                    'obstacle_detected_subscriber',
                    'precision_stop_result_subscriber',
                    # 'roscar_status_subscriber',
                    # 'sensor_subscriber',
                    'task_complete_subscriber',
                    'task_progress_subscriber'
                ]
            ],

            ### main_ctl_service를 받아야 실행 가능
            # # service (Manager) 
            # *[
            #     f'{name} = {base_path}.service.{name}:main' for name in [
            #         'log_query_service',
            #         'manager_login_service',w
            #         'roscar_status_service',
            #     ]
            # ],

            # 주행용 플래너 노드
            *[
                f'{name} = {base_path}.drive.{name}:main' for name in [
                    # 'a_star_planner',
                    'global_planner',
                    'path_follower',
                ]
            ]
        ],
    },
)