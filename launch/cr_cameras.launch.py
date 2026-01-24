import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("cr_camera_driver")
    default_config_file = os.path.join(pkg_share, "config", "cameras.yaml")
    init_script = os.path.join(pkg_share, "scripts", "init_cameras.sh")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="Path to cameras.yaml",
            ),
            DeclareLaunchArgument(
                "enable_h265",
                default_value="true",
                description="Enable H265 compression for all cameras",
            ),
            ExecuteProcess(
                cmd=["bash", "-c", f"{init_script} --no-gstreamer"],
                name="camera_init",
                output="screen",
            ),
            Node(
                package="cr_camera_driver",
                executable="cr_camera_node",
                name="cr_camera_node",
                output="screen",
                parameters=[
                    {"config_file": LaunchConfiguration("config_file")},
                    {"enable_h265": LaunchConfiguration("enable_h265")},
                ],
            ),
        ]
    )
