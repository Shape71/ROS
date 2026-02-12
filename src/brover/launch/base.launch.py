from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("brover")).resolve()

    cyphal_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("cyphal_ros2_bridge"), "/launch/cyphal_bridge_launch.py"]
        ),
        launch_arguments=[("config_file", str(pkg_share / "config" / "cyphal_bridge_base.json"))],
    )

    return LaunchDescription(
        [
            # Hardware comms
            cyphal_bridge_launch,
            # IMU
            Node(
                package="bhi360_imu_node",
                executable="bhi360_imu_node",
                remappings=[("/bhi360/imu", "/imu")],
            ),
            # Joystick
            Node(
                package="joy",
                executable="joy_node",
            ),
        ]
    )
