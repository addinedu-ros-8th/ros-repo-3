from setuptools import find_packages, setup

package_name = 'domain_bridge_controller'

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
    maintainer='pinky',
    maintainer_email='hinoonyaso@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launch_domain_bridge = domain_bridge_controller.launch_domain_bridge:main',
            'domain_bridge_manager = domain_bridge_controller.domain_bridge_manager:main',
        ],
    },
)
