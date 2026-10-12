import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    # Get the package directory
    bringup_dir = get_package_share_directory("hik_camera_ros2_driver")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    # Create the launch configuration variables
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    # Declare the launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "camera_params.yaml"),
        description="The joystick configuration file path",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    start_hik_camera_cmd = Node(
        name="hik_camera_ros2_driver",
        package="hik_camera_ros2_driver",
        executable="hik_camera_ros2_driver_node",
        parameters=[params_file],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch arguments
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch the nodes
    ld.add_action(start_hik_camera_cmd)

    return ld
