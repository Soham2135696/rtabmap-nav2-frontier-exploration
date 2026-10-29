import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def package_available(name):
    try:
        get_package_share_directory(name)
        return True
    except PackageNotFoundError:
        return False


# mode:=sim (default) -> Gazebo rover, unchanged from the original launch file
SIM_ARGS = {
    # Camera topics from rover.urdf depth camera plugin
    'rgb_topic':          '/camera/camera/image_raw',
    'depth_topic':        '/camera/camera/depth/image_raw',
    'camera_info_topic':  '/camera/camera/camera_info',

    # Odometry published by diff_drive plugin
    'odom_topic':         '/odom',

    # Frames
    'frame_id':       'base_footprint',
    'map_frame_id':   'map',
    'odom_frame_id':  'odom',
    'publish_tf_map': 'true',

    # Sync: approx because gait vibrations cause timestamp jitter
    'approx_sync':        'true',
    'approx_sync_max_interval': '0.1',
    'queue_size':         '30',

    # Use Gazebo sim clock
    'use_sim_time':       'true',

    # RTAB-Map parameters must go through 'args'; rtabmap_launch
    # silently ignores unknown launch arguments like 'Grid/FromDepth'.
    'args': (
        # Delete DB on each run so map starts fresh every launch
        '--delete_db_on_start '
        # Add map nodes at up to 2 Hz (default 1 Hz leaves gaps when moving)
        '--Rtabmap/DetectionRate 2 '
        # Grid map for 2D occupancy (good for maze mapping)
        # Build the grid from the depth camera (0=scan, 1=depth, 2=both);
        # replaces Grid/FromDepth in RTAB-Map 0.23
        '--Grid/Sensor 1 '
        '--Grid/RayTracing true '
        '--Grid/3D false '
        # Rover drives on a flat floor, so only solve for x, y, yaw
        '--Reg/Force3DoF true '
        '--RGBD/LinearUpdate 0.05 '
        '--RGBD/AngularUpdate 0.05 '
        # /odom is Gazebo ground truth; don't let camera matching on the
        # plain, repetitive maze walls override it
        '--RGBD/NeighborLinkRefining false '
        '--RGBD/ProximityBySpace false '
        # Disable loop closure detection: identical maze corridors
        # produced false closures ~2.85 m off from ground-truth odom,
        # which duplicated and rotated the walls
        '--Kp/MaxFeatures -1'
    ),

    # Open rtabmap's own 3D visualiser
    'rtabmap_viz':        'true',

    # Don't open extra rviz (use Gazebo's rviz already running)
    'rviz':               'false',

    # RGB-D mode (not stereo, not lidar-only)
    'depth':              'true',
    'subscribe_rgb':      'true',

    # Visual odometry from the camera itself (supplement wheel odom)
    'visual_odometry':    'false',
}


def real_args(context):
    localization = context.launch_configurations['localization'] == 'true'
    delete_db = context.launch_configurations['delete_db'] == 'true' and not localization
    use_imu = context.launch_configurations['use_imu'] == 'true'
    camera_mode = context.launch_configurations['camera_mode']

    if camera_mode == 'rgbd':
        camera_args = {
            # D435i topics from real_sensors.launch.py (depth aligned to colour)
            'rgb_topic':          '/camera/camera/color/image_raw',
            'depth_topic':        '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic':  '/camera/camera/color/camera_info',
            'depth':              'true',
            'subscribe_rgb':      'true',
        }
    elif camera_mode == 'stereo':
        camera_args = {
            # D435i IR pair from real_sensors.launch.py camera_mode:=stereo.
            # stereo_odometry replaces rgbd_odometry; Grid/Sensor 1 builds the
            # occupancy grid from stereo disparity.
            'stereo':                   'true',
            'left_image_topic':         '/camera/camera/infra1/image_rect_raw',
            'right_image_topic':        '/camera/camera/infra2/image_rect_raw',
            'left_camera_info_topic':   '/camera/camera/infra1/camera_info',
            'right_camera_info_topic':  '/camera/camera/infra2/camera_info',
            'depth':                    'false',
            'subscribe_rgb':            'false',
        }
    else:
        raise RuntimeError(f"camera_mode must be 'rgbd' or 'stereo', got '{camera_mode}'")

    return {
        **camera_args,

        # Visual odometry (rgbd_odometry or stereo_odometry) publishes /odom and
        # TF odom -> base_footprint; rtabmap reads /odom from the topic
        'visual_odometry':    'true',
        'odom_topic':         '/odom',
        'vo_frame_id':        'odom',

        # IMU from imu_filter_madgwick (real_sensors.launch.py)
        'imu_topic':          '/imu/data',
        'wait_imu_to_init':   'true' if use_imu else 'false',

        # Frames: same layout as the sim
        'frame_id':       'base_footprint',
        'map_frame_id':   'map',
        'publish_tf_map': 'true',

        # Aligned depth has the colour image's timestamp, and the two IR images
        # are hardware-synced, so exact sync works in both camera modes.
        # One sync node (rgbd_sync, or stereo_sync in stereo mode) feeds both
        # odometry and rtabmap.
        'rgbd_sync':          'true',
        'approx_rgbd_sync':   'false',
        'approx_sync':        'false',

        'use_sim_time':       'false',
        'localization':       'true' if localization else 'false',
        'database_path':      '~/.ros/rtabmap_real.db',

        'args': (
            ('--delete_db_on_start ' if delete_db else '')
            + '--Rtabmap/DetectionRate 2 '
            # 2D occupancy grid from depth for Nav2
            '--Grid/Sensor 1 '
            '--Grid/RayTracing true '
            '--Grid/3D false '
            # D435i depth gets noisy beyond ~4 m
            '--Grid/RangeMax 3.5 '
            # Heights in base_footprint (floor = 0): below 5 cm is ground,
            # above 1 m is ignored
            '--Grid/MaxGroundHeight 0.05 '
            '--Grid/MaxObstacleHeight 1.0 '
            # Ground rover: only solve for x, y, yaw (odometry and map)
            '--Reg/Force3DoF true'
            # Loop closure, neighbour refining and proximity links stay ON
            # (defaults): real scenes have texture and odometry is not perfect
        ),
        # Restart visual odometry right after it loses tracking
        'odom_args':          '--Odom/ResetCountdown 1',

        'rtabmap_viz':        context.launch_configurations['rtabmap_viz'],
        'rviz':               'false',
    }


def launch_setup(context, *args, **kwargs):
    mode = context.launch_configurations['mode']
    if mode == 'sim':
        launch_args = SIM_ARGS
    elif mode == 'real':
        launch_args = real_args(context)
    else:
        raise RuntimeError(f"mode must be 'sim' or 'real', got '{mode}'")

    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments=launch_args.items()
    )
    return [rtabmap]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim',
                              description="'sim' (Gazebo) or 'real' (D435i)"),
        # The arguments below only apply to mode:=real
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localize on the saved map instead of mapping'),
        DeclareLaunchArgument('delete_db', default_value='true',
                              description='Start a fresh map (ignored when localization:=true)'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        DeclareLaunchArgument('use_imu',
                              default_value='true' if package_available('imu_filter_madgwick')
                              else 'false',
                              description='Match real_sensors.launch.py use_imu'),
        DeclareLaunchArgument('camera_mode', default_value='rgbd',
                              description="'rgbd' or 'stereo'; match real_sensors.launch.py"),
        OpaqueFunction(function=launch_setup),
    ])
