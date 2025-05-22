from setuptools import find_packages, setup

package_name = 'manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'PyQt6',
                      'PyQt6-sip',
                      'PyQt6-Qt6',
                      'shared_interfaces'],
    zip_safe=True,
    maintainer='sang',
    maintainer_email='hinoonyaso@gmail.com',
    description='ros package for Manager GUI',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main = manager.view.main:main',
        ],
    },
)
