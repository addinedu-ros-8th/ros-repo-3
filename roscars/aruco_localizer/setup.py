from setuptools import setup

package_name = 'aruco_localizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # 🔸 이 줄 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ArUco 마커 기반 로봇 위치 추정',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_localizer_node = aruco_localizer.aruco_localizer_node:main',
        ],
    },
)
