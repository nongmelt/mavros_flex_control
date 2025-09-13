from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'sampling_rate',
            default_value='50.0',
            description='Sampling rate for flex sensor processing'
        ),
        DeclareLaunchArgument(
            'cutoff_frequency',
            default_value='0.03',
            description='Butterworth filter cutoff frequency'
        ),
        DeclareLaunchArgument(
            'butterworth_deg',
            default_value='1',
            description='Butterworth filter order'
        ),
        DeclareLaunchArgument(
            'enable_drift_compensation',
            default_value='false',
            description='Enable drift compensation'
        ),
        DeclareLaunchArgument(
            'enable_gaussian_filter',
            default_value='true',
            description='enable_gaussian_filter'
        ),
        
        # Flex Sensor Reader Node
        Node(
            package='flex_sensor_reader', 
            executable='flex_sensor_node',
            name='flex_sensor_node',
            output='screen',
            parameters=[
                {'reference_voltage': 3.3},
                {'divider_resistance': 47000.0},
            ],
        ),

        # Flex Sensor Processor Node
        Node(
            package='flex_sensor_reader', 
            executable='flex_process',
            name='flex_sensor_processor',
            output='screen',
            parameters=[
                {'input_topic': 'flex_sensors/raw'},
                {'output_topic': 'flex_sensors/processed'},
                {'buffer_size': 30},
                {'sampling_rate': LaunchConfiguration('sampling_rate')},
                {'butterworth_deg': LaunchConfiguration('butterworth_deg')},
                {'cutoff_frequency': LaunchConfiguration('cutoff_frequency')},
                {'enable_calibration': True},
                {'calibration_samples': 1000},
                {'enable_filter': True},
                {'enable_gaussian_filter': LaunchConfiguration('enable_gaussian_filter')},
                {'enable_drift_compensation': LaunchConfiguration('enable_drift_compensation')},
                {'drift_update_rate': 0.005},
                {'drift_threshold': 50.0},
                {'drift_window_size': 50},
                # {'median_window': 3},
                # {'median_filter_size': 10},
            ]
        )
    ])