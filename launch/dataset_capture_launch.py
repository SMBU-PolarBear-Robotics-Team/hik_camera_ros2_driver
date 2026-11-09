import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    bringup_dir = get_package_share_directory("hik_camera_ros2_driver")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "dataset_capture_params.yaml"
        ),
        description="Dataset capture parameter file path",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    start_capture_cmd = Node(
        name="dataset_capture",
        package="hik_camera_ros2_driver",
        executable="dataset_capture_node.py",
        parameters=[params_file],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        emulate_tty=True,
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(start_capture_cmd)
    return ld
