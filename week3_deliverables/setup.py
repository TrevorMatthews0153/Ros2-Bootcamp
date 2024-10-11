from setuptools import find_packages, setup

package_name = 'week3_deliverables'

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
    maintainer='Trevor',
    maintainer_email='tmatthew@ucsd.edu',
    description='SAS Lab ROS2 Bootcamp Week 3 vacuum and random goal scripts<',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'spiral_pub = week3_deliverables.week3_spiral_publisher:main',
        'random_goal_pub = week3_deliverables.week3_random_goal_publisher:main',
        'random_goal_sub = week3_deliverables.week3_random_goal_subscriber:main'
        ],
    },
)
