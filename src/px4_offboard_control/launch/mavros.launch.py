from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'vicon_hostname',
            default_value='192.168.0.50:801',
            description='VICON DataStream server hostname:port'
        ),
        DeclareLaunchArgument(
            'drone_name', 
            default_value='whikserdrone',
            description='Drone object name in VICON Tracker'
        ),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyAMA0:921600',
            description='FCU connection URL'
        ),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='udp://:14550@127.0.0.1:14551',
            description='GCS connection URL'
        ),
        DeclareLaunchArgument(
            'max_prediction_ms',
            default_value='25',
            description='VICON max prediction in milliseconds'
        ),
        DeclareLaunchArgument(
            'update_rate_hz',
            default_value='100.0',
            description='VICON update rate in Hz'
        ),

        # Launch MAVROS using standard px4.launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mavros'),
                    'launch',
                    'px4.launch'
                ])
            ]),
            launch_arguments={
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
            }.items()
        ),

        # Launch VICON Bridge (original vicon_bridge package)
        Node(
            package='vicon_bridge',
            executable='vicon_bridge',
            name='vicon_bridge',
            output='screen',
            parameters=[
                {
                    'host_name': LaunchConfiguration('vicon_hostname'),
                    'max_prediction_ms': LaunchConfiguration('max_prediction_ms'),
                    'update_rate_hz': LaunchConfiguration('update_rate_hz'),
                }
            ]
        ),

        # Simple topic relay: VICON pose -> MAVROS mocap pose
        # Note: Update the source topic based on your vicon_bridge output
        # Common topics: /vicon/{drone_name}/{drone_name} or /vicon/{drone_name}/pose
        Node(
            package='topic_tools',
            executable='relay',
            name='vicon_pose_relay',
            output='screen',
            arguments=[
                ['/vicon/', LaunchConfiguration('drone_name'), '/', LaunchConfiguration('drone_name')],
                '/mavros/mocap/pose'
            ]
        ),

        # Note: For TransformStamped on /mavros/mocap/tf, you may need either:
        # 1. The custom bridge script (vicon_mocap_bridge.py), OR  
        # 2. A simple TF republisher node (see minimal_tf_relay.py below)
        # 
        # The topic relay above handles PoseStamped messages, which should be sufficient
        # for the mocap plugin to work, but TransformStamped provides better performance.

        # Static transform publishers (for original vicon_bridge)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        ),
    ])