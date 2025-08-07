# FlexSensorData.msg
# Message for flex sensor readings

std_msgs/Header header
float32[4] resistance_values  # Resistance values in ohms for 4 flex sensors
float32[4] voltage_values     # Raw voltage values from ADC
uint8 num_sensors             # Number of active sensors
string sensor_status          # Status string (e.g., "OK", "ERROR", "DISCONNECTED")