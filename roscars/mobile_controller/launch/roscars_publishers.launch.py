from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # SSID 퍼블리셔 노드
        Node(
            package='mobile_controller',          # 패키지 이름
            executable='roscar_ssid_publisher',   # 빌드된 실행파일 이름
            name='roscar_ssid_publisher',         # 런타임 노드 이름
            output='screen',
        ),
        # 배터리 퍼블리셔 노드
        Node(
            package='mobile_controller',
            executable='roscar_battery_publisher',
            name='roscar_battery_publisher',
            output='screen',
        ),
    ])
