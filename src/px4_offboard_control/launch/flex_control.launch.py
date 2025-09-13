#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Declare launch arguments with default values
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Vehicle namespace (leave empty for default)'
    )
    
    altitude_arg = DeclareLaunchArgument(
        'altitude',
        default_value='1.5',
        description='Takeoff altitude in meters'
    )
    
    takeoff_speed_arg = DeclareLaunchArgument(
        'takeoff_speed',
        default_value='1.0',
        description='Takeoff speed in m/s'
    )
    
    hover_time_arg = DeclareLaunchArgument(
        'hover_time',
        default_value='2.0',
        description='Hover time before starting flex control (seconds)'
    )
    
    # Control mode arguments
    use_velocity_control_arg = DeclareLaunchArgument(
        'use_velocity_control',
        default_value='false',
        choices=['true', 'false'],
        description='Use velocity control (true) or attitude control (false)'
    )
    
    enable_flex_control_arg = DeclareLaunchArgument(
        'enable_flex_control',
        default_value='true',
        choices=['true', 'false'],
        description='Enable flex sensor control'
    )
    
    enable_pitch_control_arg = DeclareLaunchArgument(
        'enable_pitch_control',
        default_value='true',
        choices=['true', 'false'],
        description='Enable pitch control from flex sensors'
    )
    
    enable_yaw_control_arg = DeclareLaunchArgument(
        'enable_yaw_control',
        default_value='true',
        choices=['true', 'false'],
        description='Enable yaw control from flex sensors'
    )
    
    # Attitude control limits
    max_pitch_angle_arg = DeclareLaunchArgument(
        'max_pitch_angle',
        default_value='15.0',
        description='Maximum pitch angle in degrees'
    )
    
    max_yaw_rate_arg = DeclareLaunchArgument(
        'max_yaw_rate',
        default_value='30.0',
        description='Maximum yaw rate in deg/s'
    )
    
    # Velocity control limits
    max_velocity_x_arg = DeclareLaunchArgument(
        'max_velocity_x',
        default_value='2.0',
        description='Maximum forward/backward velocity in m/s'
    )
    
    max_velocity_y_arg = DeclareLaunchArgument(
        'max_velocity_y',
        default_value='2.0',
        description='Maximum left/right velocity in m/s'
    )
    
    max_velocity_z_arg = DeclareLaunchArgument(
        'max_velocity_z',
        default_value='1.0',
        description='Maximum up/down velocity in m/s'
    )
    
    # PID controller parameters
    pitch_kp_arg = DeclareLaunchArgument(
        'pitch_kp',
        default_value='0.00015',
        description='Pitch PID proportional gain'
    )
    
    pitch_kd_arg = DeclareLaunchArgument(
        'pitch_kd',
        default_value='0.000001',
        description='Pitch PID derivative gain'
    )
    
    yaw_kp_arg = DeclareLaunchArgument(
        'yaw_kp',
        default_value='0.03',
        description='Yaw PID proportional gain'
    )
    
    yaw_kd_arg = DeclareLaunchArgument(
        'yaw_kd',
        default_value='0.001',
        description='Yaw PID derivative gain'
    )
    
    pitch_setpoint_arg = DeclareLaunchArgument(
        'pitch_setpoint',
        default_value='2000.0',
        description='Pitch control setpoint'
    )
    
    yaw_setpoint_arg = DeclareLaunchArgument(
        'yaw_setpoint',
        default_value='0.0',
        description='Yaw control setpoint'
    )
    
    # Other parameters
    hover_thrust_arg = DeclareLaunchArgument(
        'hover_thrust',
        default_value='0.5',
        description='Hover thrust value (0.0-1.0)'
    )
    
    deadzone_threshold_arg = DeclareLaunchArgument(
        'deadzone_threshold',
        default_value='150.0',
        description='Deadzone threshold for flex sensor input'
    )
    
    normal_scale_arg = DeclareLaunchArgument(
        'normal_scale',
        default_value='1.0',
        description='Scaling factor for normal flex sensor'
    )
    
    balance_scale_arg = DeclareLaunchArgument(
        'balance_scale',
        default_value='1.0',
        description='Scaling factor for balance flex sensor'
    )

    # Main flex offboard control node
    flex_control_node = Node(
        package='px4_offboard_control',  # Replace with your actual package name
        executable='offboard_control',  # Replace with your executable name
        name='offboard_control',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'namespace': LaunchConfiguration('namespace'),
            'altitude': LaunchConfiguration('altitude'),
            'takeoff_speed': LaunchConfiguration('takeoff_speed'),
            'hover_time': LaunchConfiguration('hover_time'),
            'use_velocity_control': LaunchConfiguration('use_velocity_control'),
            'enable_flex_control': LaunchConfiguration('enable_flex_control'),
            'enable_pitch_control': LaunchConfiguration('enable_pitch_control'),
            'enable_yaw_control': LaunchConfiguration('enable_yaw_control'),
            'max_pitch_angle': LaunchConfiguration('max_pitch_angle'),
            'max_yaw_rate': LaunchConfiguration('max_yaw_rate'),
            'max_velocity_x': LaunchConfiguration('max_velocity_x'),
            'max_velocity_y': LaunchConfiguration('max_velocity_y'),
            'max_velocity_z': LaunchConfiguration('max_velocity_z'),
            'pitch_kp': LaunchConfiguration('pitch_kp'),
            'pitch_kd': LaunchConfiguration('pitch_kd'),
            'yaw_kp': LaunchConfiguration('yaw_kp'),
            'yaw_kd': LaunchConfiguration('yaw_kd'),
            'pitch_setpoint': LaunchConfiguration('pitch_setpoint'),
            'yaw_setpoint': LaunchConfiguration('yaw_setpoint'),
            'hover_thrust': LaunchConfiguration('hover_thrust'),
            'deadzone_threshold': LaunchConfiguration('deadzone_threshold'),
            'normal_scale': LaunchConfiguration('normal_scale'),
            'balance_scale': LaunchConfiguration('balance_scale'),
        }],
        remappings=[
            # Remap topics if needed - these are examples
            # ('flex_sensor/processed', '/your_custom_flex_topic'),
        ]
    )

    return LaunchDescription([
        # Launch arguments
        namespace_arg,
        altitude_arg,
        takeoff_speed_arg,
        hover_time_arg,
        use_velocity_control_arg,
        enable_flex_control_arg,
        enable_pitch_control_arg,
        enable_yaw_control_arg,
        max_pitch_angle_arg,
        max_yaw_rate_arg,
        max_velocity_x_arg,
        max_velocity_y_arg,
        max_velocity_z_arg,
        pitch_kp_arg,
        pitch_kd_arg,
        yaw_kp_arg,
        yaw_kd_arg,
        pitch_setpoint_arg,
        yaw_setpoint_arg,
        hover_thrust_arg,
        deadzone_threshold_arg,
        normal_scale_arg,
        balance_scale_arg,
        
        # Nodes
        flex_control_node,
    ])


# Additional launch configurations for different scenarios
def generate_velocity_control_launch():
    """Launch configuration optimized for velocity control"""
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('altitude', default_value='1.5'),
        DeclareLaunchArgument('use_velocity_control', default_value='true'),
        DeclareLaunchArgument('max_velocity_x', default_value='1.5'),
        DeclareLaunchArgument('max_velocity_y', default_value='1.5'),
        DeclareLaunchArgument('pitch_kp', default_value='0.0002'),
        DeclareLaunchArgument('yaw_kp', default_value='0.06'),
        
        Node(
            package='px4_offboard_control',
            executable='offboard_control',
            name='offboard_control',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'use_velocity_control': LaunchConfiguration('use_velocity_control'),
                'altitude': LaunchConfiguration('altitude'),
                'max_velocity_x': LaunchConfiguration('max_velocity_x'),
                'max_velocity_y': LaunchConfiguration('max_velocity_y'),
                'pitch_kp': LaunchConfiguration('pitch_kp'),
                'yaw_kp': LaunchConfiguration('yaw_kp'),
            }]
        )
    ])


def generate_attitude_control_launch():
    """Launch configuration optimized for attitude control"""
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('altitude', default_value='1.5'),
        DeclareLaunchArgument('use_velocity_control', default_value='false'),
        DeclareLaunchArgument('max_pitch_angle', default_value='12.0'),
        DeclareLaunchArgument('max_yaw_rate', default_value='25.0'),
        DeclareLaunchArgument('pitch_kp', default_value='0.00015'),
        DeclareLaunchArgument('yaw_kp', default_value='0.05'),
        
        Node(
            package='px4_offboard_control',
            executable='offboard_control',
            name='offboard_control',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'use_velocity_control': LaunchConfiguration('use_velocity_control'),
                'altitude': LaunchConfiguration('altitude'),
                'max_pitch_angle': LaunchConfiguration('max_pitch_angle'),
                'max_yaw_rate': LaunchConfiguration('max_yaw_rate'),
                'pitch_kp': LaunchConfiguration('pitch_kp'),
                'yaw_kp': LaunchConfiguration('yaw_kp'),
            }]
        )
    ])