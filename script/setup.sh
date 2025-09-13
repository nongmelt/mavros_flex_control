ros2 launch mavros px4.launch fcu_url:=/dev/ttyAMA0:921600 gcs_url:=udp://:14550@192.168.0.150:14550

ros2 run vicon_bridge vicon_bridge --ros-args -p host_name:="192.168.0.50:801" -r /vicon/whiskerdrone/whiskerdrone/pose:=/mavros/vision_pose/pose -p target_subject_name:=whiskerdrone -p target_segment_name:=whiskerdrone -p tf_namespace:=vicon -p publish_specific_segment:=true -p update_rate_hz:=60.0

ros2 launch flex_sensor_reader flex_sensors.launch.py enable_gaussian_filter:=false cutoff_frequency:=0.8
ros2 bag record -a -o data/stabilise_bumping_1

ros2 launch px4_offboard_control flex_control.launch.py altitude:=0.4 use_velocity_control:=true enable_yaw_control:=true deadzone_threshold:=150.0