from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    package_path = get_package_share_directory("robotic_arm")
    xacro_file = os.path.join(package_path, "urdf", "ur5.urdf.xacro")
    world_file = os.path.join(package_path, "worlds", "empty.sdf")
    controllers_file = os.path.join(package_path, "config", "controllers.yaml")
    add_gz_resource = SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=os.path.dirname(package_path)
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    set_sim_time = SetParameter(name="use_sim_time", value=use_sim_time)

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro", " ", xacro_file]),
            value_type=str
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": world_file
        }.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "robotic_arm",
            "-z", "0.05"
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster],
        )
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    load_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller],
        )
    )

    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    load_gripper_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller,
            on_exit=[gripper_controller],
        )
    )

    search_cam_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["search_cam_controller"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    load_search_cam_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller,
            on_exit=[search_cam_controller],
        )
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    cam_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/wrist_cam/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/wrist_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        output="screen",
        parameters=[{"use_sim_time" : use_sim_time}],
    )

    search_cam_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/search_cam/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/search_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        output="screen",
        parameters=[{"use_sim_time" : use_sim_time}],
    )

    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/search_lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"],
        output = "screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        set_sim_time,
        add_gz_resource,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        clock_bridge,
        cam_bridge,
        search_cam_bridge,
        lidar_bridge,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
        load_search_cam_controller,
    ])