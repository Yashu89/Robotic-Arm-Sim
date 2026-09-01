from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur5", package_name="robotic_arm_moveit")
        .to_moveit_configs()
    )
    rviz_launch = generate_moveit_rviz_launch(moveit_config)
    return LaunchDescription([rviz_launch])
