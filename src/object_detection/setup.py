from setuptools import find_packages, setup

package_name = 'object_detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yash',
    maintainer_email='yash@todo.todo',
    description='ROS2 package for autonomous object pick-and-place control using camera, LIDAR adn MoveIT ',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'search_detector = object_detection.search_detector:main',
            'pickup_controller = object_detection.pickup_controller:main',
        ],
    },
)
