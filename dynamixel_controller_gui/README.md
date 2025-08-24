# Dynamixel Controller GUI

A GUI application for testing Dynamixel motors through the `dynamixel_controller` package, providing similar functionality to the ROBOTIS Dynamixel Wizard.

## Features

- **Motor Discovery**: Scan for connected Dynamixel motors
- **Motor Control**: Basic control operations (PING, LED, Torque enable/disable)
- **Register Access**: Read and write to any motor register
- **Common Registers**: Quick access buttons for frequently used registers
- **Communication Log**: Real-time log of all communications
- **User-friendly Interface**: Intuitive GUI similar to Dynamixel Wizard

## Installation

1. Make sure you have the `dynamixel_controller` package built and installed
2. Build this package:
   ```bash
   cd ~/ros2_jazzy
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select dynamixel_controller_gui
   source install/local_setup.bash
   ```

## Usage

### Running with Real Hardware

1. Start the dynamixel_controller node:
   ```bash
   ros2 run dynamixel_controller dynamixel_controller_node --ros-args --params-file src/dynamixel_controller/config/bus_config.yaml
   ```

2. In another terminal, start the GUI:
   ```bash
   source install/local_setup.bash
   ros2 run dynamixel_controller_gui dynamixel_gui
   ```

### Testing without Hardware

For testing the GUI without actual Dynamixel motors:

1. Run the mock controller:
   ```bash
   python3 src/dynamixel_controller_gui/test_gui.py
   ```

2. In another terminal, start the GUI:
   ```bash
   source install/local_setup.bash
   ros2 run dynamixel_controller_gui dynamixel_gui
   ```

The mock controller simulates motors with IDs 1, 2, and 5.

## GUI Components

### Motor Discovery
- **Scan Motors**: Discovers all connected motors by pinging IDs 1-253
- **Ping All**: Pings all previously discovered motors
- **Found Motors**: Displays list of discovered motor IDs

### Motor Control
- **Motor ID**: Select the motor to control (1-253)
- **PING**: Test communication with selected motor
- **LED ON/OFF**: Control motor LED
- **Torque ON/OFF**: Enable/disable motor torque

### Register Access
- **Address**: Register address to read/write (0-255)
- **Length**: Number of bytes to read/write (1-8)
- **Value**: Value to write (for write operations)
- **Read/Write**: Execute read or write operation

### Common Registers
Quick access buttons for frequently used registers:
- Present Position, Velocity, Current, Temperature
- Goal Position, Velocity
- Torque Enable, LED

### Communication Log
Real-time display of all sent commands and received responses with timestamps.

## Protocol Support

The GUI supports all commands available in the `dynamixel_controller`:
- PING: Test motor communication
- READ_DATA: Read register values
- WRITE_DATA: Write register values
- SYNC_READ/SYNC_WRITE: Multi-motor operations (future enhancement)

## Register Addresses

Common register addresses (defined in DynamixelController.msg):
- Present Position: 132 (4 bytes)
- Present Velocity: 128 (4 bytes)
- Present Current: 126 (2 bytes)
- Present Temperature: 146 (1 byte)
- Goal Position: 116 (4 bytes)
- Goal Velocity: 104 (4 bytes)
- Torque Enable: 64 (1 byte)
- LED: 65 (1 byte)

## Troubleshooting

1. **GUI doesn't start**: Make sure tkinter is installed (`sudo apt install python3-tk`)
2. **No response from motors**: Check that dynamixel_controller is running and motors are connected
3. **Permission errors**: Make sure your user has access to the serial port (`sudo usermod -a -G dialout $USER`)

## Dependencies

- Python 3
- tkinter (usually included with Python)
- rclpy
- std_msgs
- dynamixel_controller