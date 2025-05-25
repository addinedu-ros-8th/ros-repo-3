from setuptools import setup

package_name = 'video_sender'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/camera_calibration.npz']),
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
            'video_streamer = video_sender.video_streamer:main',
            'aruco_align = video_sender.aruco_align:main',
            'camera = video_sender.camera:main'
        ],
    },
)
