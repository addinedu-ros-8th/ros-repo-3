from setuptools import setup

package_name = 'video_sender'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vision_udp.launch.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='채훈 심',
    maintainer_email='tlacogns@gmail.com',
    description='카메라 영상을 UDP로 AI 서버로 전송하고, ArUco 마커는 로컬에서 인식하는 ROS2 패키지',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_aruco_node = video_sender.camera_aruco_node:main',
            'udp_streamer_node = video_sender.udp_streamer_node:main',
        ],
    },
)
