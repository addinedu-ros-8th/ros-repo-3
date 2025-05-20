from setuptools import setup
import os

package_name = 'main_service'

setup(
    name=package_name,
    version='0.0.1',
    # ros_interface 폴더를 루트로 삼아, 하위 패키지들을 나열
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
        (os.path.join('share', package_name, 'config'), ['config/goal_position.json']),
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
            'start_delivery_client     = ros_interface.action.start_delivery_client:main',
            'move_to_goal_client       = ros_interface.action.move_to_goal_client:main',
            'log_query_service         = ros_interface.service.log_query_service:main',
            'log_event_publisher       = ros_interface.publisher.log_event_publisher:main',
            'control_command_publisher = ros_interface.publisher.control_command_publisher:main',
            'sensor_listener_node      = ros_interface.subscriber.sensor_listener_node:main',
        ],
    },
)
