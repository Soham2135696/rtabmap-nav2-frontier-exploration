import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('depth')
    urdf_path = os.path.join(package_dir, 'urdf', 'rover.urdf')

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    robot_description = robot_description.replace(
        'package://depth',
        'file://' + package_dir
    ).replace('DEPTH_SHARE_DIR', package_dir)

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        #Joint State Publisher GUI (manual sliders)
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # ),
        # Node(
        #     package='depth',
        #     executable='ik_control',
        #     name='ik_control',
        #     output='screen'
        # ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
