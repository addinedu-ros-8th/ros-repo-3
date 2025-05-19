from setuptools import setup

package_name = 'roscar_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='사용자이름',
    maintainer_email='example@email.com',
    description='My robot goal navigation sender',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_sender = roscar_nav.goal_sender:main',
            'move_to_goal_server = roscar_nav.move_to_goal_server:main',
        ],
    },
)
