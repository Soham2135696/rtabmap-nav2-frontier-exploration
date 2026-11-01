import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node


# robot:=rover      wheeled base + quadruped body (default); diff_drive drives it from /cmd_vel
# robot:=quadruped  quadruped alone on the ground
ROBOTS = {
    'rover': {'urdf': 'rover.urdf', 'z': '0.15', 'yaw': '0.0'},
    # feet are 0.165 m below base_link (body centre) at the zero joint pose
    'quadruped': {'urdf': 'quadruped.urdf', 'z': '0.18', 'yaw': '0.0'},
}


def launch_setup(context):
    robot = context.launch_configurations['robot']
    if robot not in ROBOTS:
        raise RuntimeError(f"robot must be one of {sorted(ROBOTS)}, got '{robot}'")
    gui = context.launch_configurations['gui'] == 'true'

    package_dir = get_package_share_directory('depth')
    urdf_path = os.path.join(package_dir, 'urdf', ROBOTS[robot]['urdf'])

    with open(urdf_path, 'r') as infp:
        urdf_content = infp.read()

    urdf_content = urdf_content.replace(
        'package://depth',
        'file://' + package_dir
    ).replace('DEPTH_SHARE_DIR', package_dir)

    actions = [

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            #name='rsp',
            output='screen',
            parameters=[
                {'robot_description': urdf_content},
                {'use_sim_time': True}
            ]
        ),

        ExecuteProcess(
            cmd=[
                'gazebo' if gui else 'gzserver',
                '--verbose',
                os.path.join(package_dir, 'worlds', 'maze.world'),
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', robot,
                '-x', '-4.0',
                '-y', '-4.0',
                '-z', ROBOTS[robot]['z'],
                '-Y', ROBOTS[robot]['yaw']
            ]
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_group_controller"],
            output="screen",
        ),

    ]

    if gui:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='rover',
                              description="'rover' (wheels + quadruped) or 'quadruped' (alone)"),
        DeclareLaunchArgument('gui', default_value='true',
                              description="'false' runs gzserver only, without RViz"),
        OpaqueFunction(function=launch_setup),
    ])
