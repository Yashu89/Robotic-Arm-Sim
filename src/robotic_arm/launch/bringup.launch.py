import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    arm = get_package_share_directory("robotic_arm")
    moveit = get_package_share_directory("robotic_arm_moveit")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm, "launch", "gazebo.launch.py")
        )
    )

    move_grp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit, "launch", "move_group.launch.py")
        )
    )

    moveit_rvizz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit, "launch", "moveit_rviz.launch.py")
        )
    )

    return LaunchDescription([
        gazebo,
        move_grp,
        moveit_rvizz,
    ])