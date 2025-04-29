from setuptools import find_packages, setup

package_name = 'test_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sang',
    maintainer_email='hinoonyaso@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_node = main_server.ros_nodes.service_node:main',
            'controller_node = mobile_controller.ros_nodes.controller_node:main',
            'send_waypoints_node = main_server.ros_nodes.send_waypoints_node:main',
            'sequential_navigate_node = main_server.ros_nodes.sequential_navigate_node:main',
        ],
    },
)
