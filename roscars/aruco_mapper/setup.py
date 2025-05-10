from setuptools import setup

package_name = 'aruco_mapper'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mapper.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='심채훈',
    maintainer_email='your@email.com',
    description='ArUco 마커의 map 기준 좌표를 저장하는 도구',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_recorder = aruco_mapper.marker_recorder:main',
            'camera_publisher = aruco_mapper.camera_publisher:main',
            'aruco_localizer_node = aruco_mapper.aruco_localizer_node:main',
        ],
    },
)
