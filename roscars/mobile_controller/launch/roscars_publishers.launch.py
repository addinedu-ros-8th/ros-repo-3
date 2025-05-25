from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # SSID 퍼블리셔 노드
        Node(
            package='mobile_controller',
            executable='roscar_ssid_publisher',
            name='roscar_ssid_publisher',
            output='screen',
        ),
        # 배터리 퍼블리셔 노드
        Node(
            package='mobile_controller',
            executable='roscar_battery_publisher',
            name='roscar_battery_publisher',
            output='screen',
        ),
        # IMU 퍼블리셔 노드
        Node(
            package='mobile_controller',
            executable='roscar_imu_publisher',
            name='roscar_imu_publisher',
            output='screen',
        ),
        # 라이다 퍼블리셔 노드
        Node(
            package='mobile_controller',
            executable='roscar_lidar_publisher',
            name='roscar_lidar_publisher',
            output='screen',
        ),
        # 레지스터 퍼블리셔 노드
        Node(
            package='mobile_controller',
            executable='roscar_register_publisher',
            name='roscar_register_publisher',
            output='screen',
        ),
        # 작업 완료 퍼블리셔 노드
        Node(
            package='mobile_controller',
            executable='roscar_task_complete_publisher',
            name='roscar_task_complete_publisher',
            output='screen',
        ),
        # 작업 진행 퍼블리셔 노드
        Node(
            package='mobile_controller',
            executable='roscar_task_progress_publisher',
            name='roscar_task_progress_publisher',
            output='screen',
        ),
        # 초음파 퍼블리셔 노드
        Node(
            package='mobile_controller',
            executable='roscar_ultra_publisher',
            name='roscar_ultra_publisher',
            output='screen',
        ),
    ])