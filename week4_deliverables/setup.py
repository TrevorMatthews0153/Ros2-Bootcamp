from setuptools import find_packages, setup

package_name = 'week4_deliverables'

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
    maintainer='ubuntu',
    maintainer_email='tmatthew@ucsd.edu',
    description='Week4 Deliverables SAS Lab ROS2 Bootcamp',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'definedgoal_pub = week4_deliverables.week4_square_publisher:main',
        'spiral_pub = week4_deliverables.week4_spiral_publisher:main'
        ],
    },
)
