from setuptools import setup

package_name = 'aruco_mapper'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ROS 2 패키지 색인
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 패키지 루트 (package.xml + JSON/NPZ)
        ('share/' + package_name, [
            'package.xml',
            'aruco_marker_positions.json',
            'camera_calibration.npz',
        ]),
        # 런치 파일
        ('share/' + package_name + '/launch', [
            'launch/aruco_mapping.launch.py',
        ]),
    ],
    install_requires=[
        'setuptools',
        'transforms3d',
        'numpy',           # tvec 연산용
        'opencv-python',   # cv2
    ],
    zip_safe=True,
    maintainer='심채훈',
    maintainer_email='your@email.com',
    description='ArUco 마커의 map 기준 좌표를 저장하는 도구',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aruco_localizer_node = aruco_mapper.aruco_localizer_node:main',
            'marker_recorder        = aruco_mapper.marker_recorder:main',
            'aruco_initialpose_publisher = aruco_mapper.initialpose_custom:main',
        ],
    },
)
