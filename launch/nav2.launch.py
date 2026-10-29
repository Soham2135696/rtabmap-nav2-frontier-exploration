import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = context.launch_configurations['mode']
    if mode not in ('sim', 'real'):
        raise RuntimeError(f"mode must be 'sim' or 'real', got '{mode}'")

    # explore:=auto explores in sim only: the real rover has no motor driver yet
    explore = context.launch_configurations['explore']
    if explore not in ('auto', 'true', 'false'):
        raise RuntimeError(f"explore must be 'auto', 'true' or 'false', got '{explore}'")
    if explore == 'auto':
        explore = 'true' if mode == 'sim' else 'false'

    package_dir = get_package_share_directory('depth')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # sim: costmaps read /camera/camera/points from Gazebo
    # real: costmaps read /camera/obstacle_cloud from real_sensors.launch.py
    params_file = 'nav2_params.yaml' if mode == 'sim' else 'nav2_params_real.yaml'

    # Nav2 without map_server/amcl: RTAB-Map provides /rtabmap/map and map -> odom.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True' if mode == 'sim' else 'False',
            'params_file': os.path.join(package_dir, 'config', params_file),
        }.items()
    )
    if explore == 'false':
        return [navigation]

    # Frontier exploration: waits for Nav2 and /rtabmap/map itself, then sends
    # NavigateToPose goals until no frontier is left (then the node exits).
    # Keep teleop closed while it runs: both publish /cmd_vel.
    explorer = Node(
        package='depth', executable='frontier_explorer',
        name='frontier_explorer', output='screen',
        parameters=[{'use_sim_time': mode == 'sim'}],
    )
    return [navigation, explorer]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim',
                              description="'sim' (Gazebo) or 'real' (D435i)"),
        DeclareLaunchArgument('explore', default_value='auto',
                              description="Frontier exploration: 'auto' (on in sim, off in "
                                          "real), 'true' or 'false'"),
        OpaqueFunction(function=launch_setup),
    ])
