# ROS2 Pi CNC Controller

A ROS 2 workspace for controlling CNC machines using a Raspberry Pi. This project provides nodes for joystick-based CNC teleop control, serial communication with Arduino/GRBL controllers, and system monitoring utilities.

---

## Features

- **Joystick Teleoperation** — Control your CNC machine in real-time using a gamepad
- **Serial Communication** — Direct GRBL command interface via serial port
- **LED Control** — ROS-based LED control through Arduino
- **Temperature Monitoring** — Real-time Raspberry Pi CPU temperature publisher
- **Bringup Launch** — Single launch file to start all essential nodes

---

## Packages

| Package | Description |
|---------|-------------|
| `pi_firmware` | Core CNC control nodes (joystick teleop, serial commands, LED control) |
| `pi_bringup` | Launch files for bringing up the system |
| `pi_interfaces` | Custom ROS 2 service definitions |
| `pi_tests` | Test nodes and utilities (temperature monitoring) |

---

## Prerequisites

- **ROS 2 Jazzy**
- **Python 3.10+**
- **Raspberry Pi** (tested on Pi 4)
- **GRBL-compatible CNC controller** (e.g., Arduino with GRBL firmware)
- **Joy package** for joystick input:
  ```bash
  sudo apt install ros-jazzy-joy
  ```
- Python packages:
  ```bash
  pip install pyserial gpiozero
  ```

---

## Installation

1. **Clone the repository** into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/ros-pi-cnc.git
   ```

2. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace**:
   ```bash
   colcon build --symlink-install
   ```

4. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

---

## Usage

### Joystick CNC Control
Control the CNC machine using a joystick. First, launch the joy node, then run the teleop controller.

**Terminal 1** — Start the joystick driver:
```bash
ros2 run joy joy_node
```

**Terminal 2** — Start the CNC teleop:
```bash
ros2 run pi_firmware joy_control_cnc
```

### Serial Command Interface
Send raw G-code commands to the CNC controller via the `serial_command` topic.
```bash
ros2 run pi_firmware serial_command
```
```bash
# Example: Send a G-code command
ros2 topic pub /serial_command std_msgs/msg/String "data: 'G1 X1.0 Y1.0 F10'"
```

### LED Control
Toggle LED on/off via the `led_control` topic.
```bash
ros2 run pi_firmware led_control
```
```bash
# Turn LED ON
ros2 topic pub /led_control std_msgs/msg/Bool "data: true"
```

### Temperature Monitor
Publishes Raspberry Pi CPU temperature to `/temperature` topic every second.
```bash
ros2 run pi_tests check_temperature
```



---

## Topic Reference

### Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/joy` | `sensor_msgs/Joy` | Joystick input (subscribed by `joy_control_cnc`) |
| `/serial_command` | `std_msgs/String` | Raw G-code commands to CNC |
| `/led_control` | `std_msgs/Bool` | LED on/off control |
| `/temperature` | `example_interfaces/Float64` | CPU temperature readings |


---

## Hardware Configuration

### CNC Machine
This project is designed for a DIY 3-axis CNC built with:
- **Controller**: Arduino Uno/Nano with GRBL firmware
- **Stepper Drivers**: 3× A4988 motor drivers
- **Motors**: 3× CD-ROM/DVD stepper motors
- **Axes**: X, Y, Z (recycled from optical drives)

> **Note**: You may need to adjust serial port paths in the source files or add udev rules for persistent device naming.

### Joystick Mapping
- **Left Stick X-axis** (`axes[0]`) → CNC X movement
- **Left Stick Y-axis** (`axes[1]`) → CNC Y movement
- **Deadzone**: 0.2 (configurable in code)

---

## Project Structure

```
ros-pi-cnc/
├── pi_bringup/
│   └── launch/
│       └── bringup.launch.py    # System launch file
├── pi_firmware/
│   └── pi_firmware/
│       ├── joy_control_cnc.py   # Joystick → CNC teleop
│       ├── serial_command.py    # Serial G-code interface
│       └── led_control.py       # LED control node
└── pi_tests/
    └── pi_tests/
        └── check_temperature.py     # CPU temp publisher
```

---

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

## License

This project is licensed under the [MIT License](LICENSE).

---

## Author

**Yacin** — [yacin.ha9@gmail.com](mailto:yacin.ha9@gmail.com)
