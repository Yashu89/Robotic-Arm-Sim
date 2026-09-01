from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    search_detector = Node(
        package="object_detection",
        executable="search_detector",
        name="search_detector",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    pickup_controller = Node(
        package="object_detection",
        executable="pickup_controller",
        name="pickup_controller",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        search_detector,
        pickup_controller,
    ])