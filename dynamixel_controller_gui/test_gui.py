#!/usr/bin/env python3
"""
Test script for dynamixel_controller_gui
This script can be used to test the GUI without actual hardware
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from dynamixel_controller.msg import DynamixelController
import time

class MockDynamixelController(Node):
    """Mock controller that simulates dynamixel_controller responses"""
    
    def __init__(self):
        super().__init__('mock_dynamixel_controller')
        
        # Subscribe to commands
        self.tx_subscription = self.create_subscription(
            UInt8MultiArray,
            'dynamixel_tx',
            self.tx_callback,
            10
        )
        
        # Publish responses
        self.rx_publisher = self.create_publisher(UInt8MultiArray, 'dynamixel_rx', 10)
        
        # Simulate some motors present
        self.simulated_motors = [1, 2, 5]
        self.motor_positions = {1: 2048, 2: 1024, 5: 3072}  # Simulated positions
        self.motor_temperatures = {1: 25, 2: 30, 5: 28}  # Simulated temperatures
        
        self.get_logger().info('Mock Dynamixel Controller started')
        self.get_logger().info(f'Simulated motors: {self.simulated_motors}')
        
    def tx_callback(self, msg):
        """Handle incoming commands and send appropriate responses"""
        if len(msg.data) < 2:
            return
            
        instruction = msg.data[0]
        motor_id = msg.data[1]
        
        # Create response message
        response = UInt8MultiArray()
        
        if instruction == DynamixelController.PING:
            # Respond to PING only for simulated motors
            if motor_id in self.simulated_motors:
                response.data = [DynamixelController.PING, motor_id, 0]  # 0 = success
                self.rx_publisher.publish(response)
                self.get_logger().info(f'PING response sent for motor {motor_id}')
                
        elif instruction == DynamixelController.READ_DATA:
            if motor_id in self.simulated_motors and len(msg.data) >= 4:
                address = msg.data[2]
                length = msg.data[3]
                
                # Simulate different register values
                value = 0
                if address == DynamixelController.PRESENT_POSITION:
                    value = self.motor_positions[motor_id]
                elif address == DynamixelController.PRESENT_TEMPERATURE:
                    value = self.motor_temperatures[motor_id]
                elif address == DynamixelController.PRESENT_VELOCITY:
                    value = 0  # Stationary
                elif address == DynamixelController.TORQUE_ENABLE:
                    value = 1  # Enabled
                elif address == DynamixelController.LED:
                    value = 0  # OFF
                else:
                    value = 42  # Default value for unknown registers
                
                # Convert value to bytes (little-endian)
                value_bytes = []
                for i in range(length):
                    value_bytes.append((value >> (8 * i)) & 0xFF)
                
                response.data = [DynamixelController.READ_DATA, motor_id, 0] + value_bytes
                self.rx_publisher.publish(response)
                self.get_logger().info(f'READ response sent for motor {motor_id}, address {address}: {value}')
                
        elif instruction == DynamixelController.WRITE_DATA:
            if motor_id in self.simulated_motors and len(msg.data) >= 4:
                address = msg.data[2]
                length = msg.data[3]
                
                # Extract value from bytes
                value = 0
                for i in range(length):
                    if i + 4 < len(msg.data):
                        value |= msg.data[i + 4] << (8 * i)
                
                # Update simulated values
                if address == DynamixelController.GOAL_POSITION:
                    self.motor_positions[motor_id] = value
                
                response.data = [DynamixelController.WRITE_DATA, motor_id, 0]  # 0 = success
                self.rx_publisher.publish(response)
                self.get_logger().info(f'WRITE response sent for motor {motor_id}, address {address}, value {value}')

def main(args=None):
    rclpy.init(args=args)
    
    mock_controller = MockDynamixelController()
    
    try:
        rclpy.spin(mock_controller)
    except KeyboardInterrupt:
        pass
    finally:
        mock_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()