#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import time

class FlexSensorDebugger(Node):
    def __init__(self):
        super().__init__('flex_sensor_debugger')
        
        # Initialize I2C and ADS1115
        try:
            try:
                self.i2c = busio.I2C(board.SCL, board.SDA)
            except AttributeError:
                self.i2c = busio.I2C(board.D3, board.D2)
            
            self.ads = ADS.ADS1115(self.i2c)
            
            # Initialize analog input channels
            self.channels = [
                AnalogIn(self.ads, ADS.P0),
                AnalogIn(self.ads, ADS.P1),
                AnalogIn(self.ads, ADS.P2),
                AnalogIn(self.ads, ADS.P3)
            ]
            
            self.get_logger().info('ADS1115 initialized for debugging')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ADS1115: {str(e)}')
            return
        
        # Parameters
        self.ref_voltage = 3.3
        self.divider_resistance = 47000.0
        
        # Start debug timer
        self.timer = self.create_timer(1.0, self.debug_sensors)
        
        self.get_logger().info('Flex sensor debugger started')
    
    def debug_sensors(self):
        print("\n" + "="*80)
        print("FLEX SENSOR DEBUG REPORT")
        print("="*80)
        
        for i, channel in enumerate(self.channels):
            try:
                # Raw readings
                voltage = channel.voltage
                raw_value = channel.value
                
                # Calculate resistance
                if voltage > 0.01:  # Avoid division by zero
                    flex_resistance = self.divider_resistance * (self.ref_voltage - voltage) / voltage
                else:
                    flex_resistance = float('inf')
                
                # Voltage divider analysis
                voltage_ratio = voltage / self.ref_voltage
                
                print(f"\nSensor {i+1} (Channel A{i}):")
                print(f"  Raw ADC Value:    {raw_value:5d} / 32767")
                print(f"  Voltage:          {voltage:.4f} V")
                print(f"  Voltage Ratio:    {voltage_ratio:.4f}")
                print(f"  Calculated R:     {flex_resistance:.0f} Ω")
                
                # Diagnosis
                if voltage < 0.1:
                    print(f"  ⚠️  WARNING: Very low voltage - possible short circuit or bad connection")
                elif voltage > 3.2:
                    print(f"  ⚠️  WARNING: Very high voltage - possible open circuit")
                elif flex_resistance > 200000:
                    print(f"  ⚠️  WARNING: Very high resistance - check sensor connection")
                elif flex_resistance < 5000:
                    print(f"  ⚠️  WARNING: Very low resistance - sensor may be damaged")
                else:
                    print(f"  ✅ OK: Normal reading")
                    
            except Exception as e:
                print(f"\nSensor {i+1}: ERROR - {str(e)}")
        
        print("\n" + "="*80)
        print("TROUBLESHOOTING GUIDE:")
        print("• 400kΩ reading = High voltage (>3V) = Open circuit or loose connection")
        print("• Check wire connections to ADS1115 and flex sensor")
        print("• Verify 47kΩ pull-down resistor is connected")
        print("• Test with multimeter: measure voltage directly at ADS1115 pins")
        print("• Flex sensor should read 10-40kΩ when straight, 50-100kΩ when bent")
        print("="*80)

def main(args=None):
    rclpy.init(args=args)
    
    debugger = FlexSensorDebugger()
    
    try:
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        pass
    finally:
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()