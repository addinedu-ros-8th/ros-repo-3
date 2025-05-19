from setuptools import setup, find_packages

package_name = 'main_service'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yonmilk',
    maintainer_email='yonmilk@icloud.com',
    description='Main service ROS2 interface package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'log_query_service = service.log_query_service:main',
            'log_event_publisher = publisher.log_event_publisher:main',
            'control_command_publisher = publisher.control_command_publisher:main',
            'start_delivery_client = action.start_delivery_client:main',
        ],
    },
)
