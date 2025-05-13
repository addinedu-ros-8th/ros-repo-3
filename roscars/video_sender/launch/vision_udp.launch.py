from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 카메라 퍼블리셔
        Node(
            package='video_sender',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        ),
        # ArUco 마커 로컬 인식
        Node(
            package='video_sender',
            executable='aruco_localizer_node',
            name='aruco_localizer',
            output='screen'
        ),
        # ArUco 마커 위치 저장기
        Node(
            package='video_sender',
            executable='marker_recorder',
            name='marker_recorder',
            output='screen'
        ),
        # AI 서버로 UDP 영상 송신
        Node(
            package='video_sender',
            executable='udp_streamer_node',
            name='udp_streamer',
            output='screen'
        )
    ])
