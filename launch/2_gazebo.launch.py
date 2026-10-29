import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


# robot:=rover      wheeled base + legs (default); diff_drive drives it from /cmd_vel
# robot:=quadruped  legs only, like the real robot; walk_gait moves the legs and
#                   ground_truth_odom publishes /odom (no wheels, no diff_drive)
ROBOTS = {
    'rover': {'urdf': 'rover.urdf', 'z': '0.15'},
    # Spawns standing: the joints start in the stand pose (initial_value in the URDF)
    'quadruped': {'urdf': 'quadruped.urdf', 'z': '0.01'},
}

# walk_gait's STAND (joint order of controllers_quadruped.yaml). Sent once when the leg
# controllers are up, so the legs hold the robot until walk_gait takes over
STAND = [0.0, -1.4002, -0.46137, 0.0, 1.4002, -0.46137,
         0.0, 1.4002, -0.46137, 0.0, -1.4002, -0.46137]


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
                '-z', ROBOTS[robot]['z']
            ]
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),

    ]

    # quadruped: joint_group_controller feeds pid_controller (chained), so pid_controller
    # must be active first; one spawner activates them in this order
    leg_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=(["pid_controller"] if robot == 'quadruped' else [])
        + ["joint_group_controller"],
        output="screen",
    )
    actions.append(leg_spawner)

    if gui:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ))

    if robot == 'quadruped':
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=leg_spawner,
            on_exit=[ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once', '/joint_group_controller/commands',
                     'std_msgs/msg/Float64MultiArray', f'{{data: {STAND}}}'],
                output='screen')])))
        actions.append(Node(
            package='depth',
            executable='ground_truth_odom',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='rover',
                              description="'rover' (wheels + legs) or 'quadruped' (legs only)"),
        DeclareLaunchArgument('gui', default_value='true',
                              description="'false' runs gzserver only, without RViz"),
        OpaqueFunction(function=launch_setup),
    ])
