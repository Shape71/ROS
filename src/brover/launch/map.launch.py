from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("brover")).resolve()

    cyphal_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("cyphal_ros2_bridge"), "/launch/cyphal_bridge_launch.py"]
        ),
        launch_arguments=[("config_file", str(pkg_share / "config" / "cyphal_bridge_base.json"))],
    )

    # Lidar t1 launch description variables
    channel_type = LaunchConfiguration('channel_type', default='udp')
    udp_ip = LaunchConfiguration('udp_ip', default='192.168.0.10')
    udp_port = LaunchConfiguration('udp_port', default='8089') 
    frame_id = LaunchConfiguration('frame_id', default='base_scan')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    # End of lidar variables

    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", str(pkg_share / "urdf" / "brover.urdf.xacro")]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_controllers = str(pkg_share / "config" / "brover_controllers.yaml")

    robot_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "brover_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            "-r /brover_controller/cmd_vel:=/cmd_vel",
        ],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    range_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "range_sensor_broadcaster"
        ],
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

        # ROS2 Control
        robot_control_node,
        robot_state_publisher,
        robot_controller_spawner,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_controller_spawner,
                on_exit=[joint_state_broadcaster],
            )
        ),
        # Lidar t1 return launch description
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'udp_ip',
            default_value=udp_ip,
            description='Specifying udp ip to connected lidar'),

        DeclareLaunchArgument(
            'udp_port',
            default_value=udp_port,
            description='Specifying udp port to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type': channel_type,
                        'udp_ip': udp_ip,
                        'udp_port': udp_port,
                        'frame_id': frame_id,
                        'inverted': inverted,
                        'angle_compensate': angle_compensate,
                        'scan_mode': scan_mode}],
            output='screen'),

        # Temp
        Node(
            package = 'slam_toolbox',
            executable='async_slam_toolbox_node',
            name='temp',
        )
        # end
    ]
    )