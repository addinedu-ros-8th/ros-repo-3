from setuptools import setup

package_name = 'video_sender'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # launch 폴더 내 모든 launch 파일(.py, .xml) 포함
        ('share/' + package_name + '/launch', [
            'launch/vision_udp.launch.xml'
        ]),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='채훈 심',
    maintainer_email='tlacogns@gmail.com',
    description='카메라 영상을 UDP로 AI 서버에 전송하고, ArUco 마커를 로컬에서 인식하는 ROS2 패키지',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_publisher = video_sender.camera_publisher:main',
            'aruco_localizer_node = video_sender.aruco_localizer_node:main',
            'marker_recorder = video_sender.marker_recorder:main',
            'udp_streamer_node = video_sender.udp_streamer_node:main',
        ],
    },
)
