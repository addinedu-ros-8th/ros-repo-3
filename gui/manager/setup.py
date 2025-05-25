import os
from setuptools import setup, find_packages

package_name = 'manager'
base_path = 'gui.manager.ros_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='.', include=['server', 'server.*']),
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), ['main.py']),  
    ],
    install_requires=[
        'setuptools',
        'PyQt6',
        'PyQt6-sip',
        'PyQt6-Qt6',
        'shared_interfaces'
    ],
    zip_safe=True,
    maintainer='sang',
    maintainer_email='hinoonyaso@gmail.com',
    description='ros package for Manager GUI',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main = gui.manager.main:main',
        ],
    },
)
