import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def package_available(name):
    try:
        get_package_share_directory(name)
        return True
    except PackageNotFoundError:
        return False


def camera_setup(context, *args, **kwargs):
    camera_mode = context.launch_configurations['camera_mode']
    if camera_mode not in ('rgbd', 'stereo'):
        raise RuntimeError(f"camera_mode must be 'rgbd' or 'stereo', got '{camera_mode}'")

    realsense_dir = get_package_share_directory('realsense2_camera')

    if camera_mode == 'rgbd':
        # D435i driver: same settings as the manual T1 command, plus depth aligned to
        # the colour camera so RTAB-Map gets registered RGB-D pairs.
        # Publishes /camera/camera/color/*, /camera/camera/aligned_depth_to_color/*,
        # /camera/camera/imu and TF camera_link -> *_optical_frame.
        rs_args = {
            'enable_gyro':                  'true',
            'enable_accel':                 'true',
            'unite_imu_method':             '2',
            'enable_sync':                  'true',
            'enable_infra1':                'false',
            'enable_infra2':                'false',
            'align_depth.enable':           'true',
            'rgb_camera.color_profile':     '640x480x15',
            'depth_module.depth_profile':   '640x480x15',
        }
        depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        depth_info_topic = '/camera/camera/aligned_depth_to_color/camera_info'
    else:
        # Stereo: RTAB-Map runs on the two global-shutter IR images
        # (/camera/camera/infra1|2/*). No colour stream and no depth alignment,
        # which saves the driver's per-frame align step. Depth stays on for the
        # Nav2 obstacle cloud; infra and depth share one sensor, so same profile.
        profile = f"640x480x{context.launch_configurations['infra_fps']}"
        rs_args = {
            'enable_gyro':                  'true',
            'enable_accel':                 'true',
            'unite_imu_method':             '2',
            'enable_sync':                  'true',
            'enable_color':                 'false',
            'enable_infra1':                'true',
            'enable_infra2':                'true',
            'align_depth.enable':           'false',
            'depth_module.infra_profile':   profile,
            'depth_module.depth_profile':   profile,
        }
        if context.launch_configurations['emitter'] == '0':
            rs_args['config_file'] = os.path.join(
                get_package_share_directory('depth'), 'config', 'realsense_emitter_off.yaml')
        depth_topic = '/camera/camera/depth/image_rect_raw'
        depth_info_topic = '/camera/camera/depth/camera_info'

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments=rs_args.items()
    )

    # Light, downsampled cloud for the Nav2 costmaps (the driver's full cloud is
    # ~300k points per frame). Published on /camera/obstacle_cloud in both modes.
    obstacle_cloud = Node(
        package='rtabmap_util', executable='point_cloud_xyz',
        name='obstacle_cloud', output='screen',
        parameters=[{
            'decimation': 4,
            'voxel_size': 0.05,
            'min_depth': 0.3,
            'max_depth': 4.0,
            'approx_sync': False,
        }],
        remappings=[
            ('depth/image', depth_topic),
            ('depth/camera_info', depth_info_topic),
            ('cloud', '/camera/obstacle_cloud'),
        ],
    )

    return [realsense, obstacle_cloud]


def generate_launch_description():

    madgwick_available = package_available('imu_filter_madgwick')

    use_imu = LaunchConfiguration('use_imu')

    args = [
        # Real mount of the D435i relative to base_link (defaults = rover.urdf camera_joint)
        DeclareLaunchArgument('cam_x', default_value='0.2'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.4'),
        DeclareLaunchArgument('cam_roll', default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),
        # Height of base_link above the floor (base_footprint), as in rover.urdf
        DeclareLaunchArgument('base_height', default_value='0.10'),
        # IMU filter needs ros-humble-imu-filter-madgwick; off automatically if missing
        DeclareLaunchArgument('use_imu',
                              default_value='true' if madgwick_available else 'false'),
        # rgbd: colour + aligned depth (default). stereo: IR left/right for RTAB-Map.
        # Must match rtabmap.launch.py camera_mode.
        DeclareLaunchArgument('camera_mode', default_value='rgbd',
                              description="'rgbd' or 'stereo'"),
        # stereo only: IR dot projector. 0 = off (better stereo odometry, weaker
        # depth on plain walls), 1 = on (better obstacle depth, worse odometry)
        DeclareLaunchArgument('emitter', default_value='0',
                              description="stereo only: IR emitter 0=off, 1=on"),
        # stereo only: IR and depth frame rate (D435i at 640x480: 6, 15, 30, 60, 90)
        DeclareLaunchArgument('infra_fps', default_value='30'),
    ]

    # Raw D435i IMU has no orientation; RTAB-Map needs it for gravity alignment
    imu_filter = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        name='imu_filter', output='screen',
        condition=IfCondition(use_imu),
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
        }],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),
            ('imu/data', '/imu/data'),
        ],
    )

    # Same frame layout as the sim: base_footprint on the floor, base_link above it.
    # rgbd_odometry publishes odom -> base_footprint.
    footprint_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['--z', LaunchConfiguration('base_height'),
                   '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
    )

    # Replaces the manual T2 (base_link -> camera_link at 0 0 0)
    camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_link_to_camera_link',
        arguments=['--x', LaunchConfiguration('cam_x'),
                   '--y', LaunchConfiguration('cam_y'),
                   '--z', LaunchConfiguration('cam_z'),
                   '--roll', LaunchConfiguration('cam_roll'),
                   '--pitch', LaunchConfiguration('cam_pitch'),
                   '--yaw', LaunchConfiguration('cam_yaw'),
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
    )

    actions = args + [OpaqueFunction(function=camera_setup),
                      imu_filter, footprint_tf, camera_tf]
    if not madgwick_available:
        actions.insert(0, LogInfo(msg=(
            'imu_filter_madgwick not found: running without IMU. '
            'Install with: sudo apt install ros-humble-imu-filter-madgwick')))

    return LaunchDescription(actions)
