from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur5", package_name="robotic_arm_moveit")
        .to_moveit_configs()
    )
    move_group_launch = generate_move_group_launch(moveit_config)
    return LaunchDescription([move_group_launch])