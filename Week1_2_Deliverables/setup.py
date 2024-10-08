from setuptools import find_packages, setup

package_name = 'Week1_2_Deliverables'

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
    description='SAS ROS2 Bootcamp deliverable scripts',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'talker = Week1_2_Deliverables.6_dim_state_publisher:main',
        'listener = Week1_2_Deliverables.6_dim_state_subscriber:main',
        'random_goal_publisher = Week1_2_Deliverables.randomgoal_publisher:main',
        'random_goal_subscriber = Week1_2_Deliverables.randomgoal_subscriber:main',
        'spiral_publisher = Week1_2_Deliverables.spiral_publisher:main',
        'spiral_subscriber = Week1_2_Deliverables.spiral_subscriber:main'
        ],
    },
)
