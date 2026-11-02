# Real quadruped, no Gazebo: robot model + leg TF + ESP serial bridge.
#
# Starts:
# - robot_state_publisher with urdf/quadruped.urdf: turns /joint_states (published by
#   `ros2 run depth walk_gait --ros-args -p real:=true`) into the leg TF and publishes
#   /robot_description for RViz
# - esp: sends /joint_states to the ESP that drives the 12 servos
# - optionally the D435i camera (real_sensors.launch.py)
#
# port:=/dev/ttyUSB0 (default)  ESP serial port
# camera:=false (default)       true: also start real_sensors.launch.py. Its
#                               arguments (camera_mode, cam_*, base_height, ...)
#                               can be passed here too, e.g. camera_mode:=stereo
# rviz:=false (default)         true: also open RViz on this machine
#
# walk_gait is not started here: it reads f / s from the keyboard, which a launch
# file cannot pass on. Start it after this launch file, so esp sees the stand-up.
#
# Mesh paths stay package://depth/..., so RViz on another machine loads the
# meshes from its own install of this package.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('depth')
    with open(os.path.join(package_dir, 'urdf', 'quadruped.urdf')) as f:
        robot_description = f.read().replace('DEPTH_SHARE_DIR', package_dir)

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='ESP serial port'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='true: also start real_sensors.launch.py'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='true: also open RViz on this machine'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='depth',
            executable='esp',
            name='serial_bridge_leg12',
            output='screen',
            parameters=[{'port': LaunchConfiguration('port')}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_dir, 'launch', 'real_sensors.launch.py')),
            condition=IfCondition(LaunchConfiguration('camera')),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
