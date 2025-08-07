from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Flex Sensor Reader Node
        Node(
            package='flex_sensor_reader', 
            executable='flex_sensor_node',  # Must match the entry point name
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
                {'buffer_size': 10},
                {'sampling_rate': 50.0},
                {'cutoff_frequency': 10.0},
                {'enable_calibration': True},
                {'calibration_samples': 1000},
                {'enable_filter': True},
                {'enable_drift_compensation': False},
                {'drift_update_rate': 0.005},
                {'drift_threshold': 50.0},
                {'drift_window_size': 50},
                {'enable_median_filter': True},
                {'median_window': 3},
                {'median_filter_size': 10},
            ]
        )
    ])
