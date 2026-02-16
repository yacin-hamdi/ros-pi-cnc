# ROS2 Pi CNC Controller

A ROS 2 workspace for controlling CNC machines using a Raspberry Pi. This project provides joystick teleoperation, an **LLM-powered natural-language commander**, a GRBL serial driver with real-time joint-state feedback, a URDF digital twin, and system monitoring utilities.

---

## Features

- **Joystick Teleoperation** — Control your CNC in real-time using a gamepad with GRBL jog commands
- **LLM Agent Commander** — Prompt an AI to draw shapes (e.g. *"draw a 20mm box in the center"*) and it generates & executes the G-code automatically via tool-use
- **GRBL Driver** — Unified serial interface supporting both velocity (joystick) and absolute (LLM) commands, with live joint-state publishing
- **Digital Twin** — URDF model of the CNC with prismatic joints, visualized in RViz2 via `robot_state_publisher`
- **Bringup Launch Files** — Single launch files for the full digital-twin stack and for the utility nodes

---

## Architecture

```
┌──────────────┐         ┌──────────────┐
│  Joystick    │────────▶│  teleop_joy  │──── /cmd_vel ────┐
│  (Gamepad)   │  /joy   │ (pi_control) │                  │
└──────────────┘         └──────────────┘                  ▼
                                                    ┌──────────────┐        ┌──────────┐
                                                    │  grbl_driver │──USB──▶│  Arduino │
                                                    │ (pi_firmware)│◀───────│  (GRBL)  │
                                                    └──────┬───────┘        └──────────┘
┌──────────────┐         ┌──────────────┐                  │
│   User       │────────▶│llm_commander │── /cnc/move_to ──┘
│   Prompt     │         │ (pi_control) │
└──────────────┘         └──────────────┘         │
                                                  ▼
                                           /joint_states
                                                  │
                                           ┌──────┴───────┐
                                           │robot_state_  │
                                           │publisher     │──▶ RViz2
                                           │(pi_description)│
                                           └──────────────┘
```

---

## Packages

| Package | Description |
|---------|-------------|
| `pi_firmware` | GRBL driver node, serial command interface, LED control |
| `pi_control` | Joystick teleop node (`teleop_joy`) and LLM agent commander (`llm_commander`) |
| `pi_description` | URDF/Xacro model of the CNC and its `robot_state_publisher` launch file |
| `pi_bringup` | Launch files: `digital_twin.launch.py` (full stack) and `bringup.launch.py` (utilities) |
| `pi_interfaces` | Custom ROS 2 service definitions (`CreateFolder.srv`) |
| `pi_tests` | Test & utility nodes (temperature monitor, folder service) |

---

## Prerequisites

- **ROS 2 Jazzy**
- **Python 3.10+**
- **Raspberry Pi** (tested on Pi 4)
- **GRBL-compatible CNC controller** (e.g., Arduino Uno/Nano with GRBL firmware)
- **Joy package** for joystick input:
  ```bash
  sudo apt install ros-jazzy-joy
  ```
- Python packages:
  ```bash
  pip install pyserial gpiozero openai openai-agents pydantic python-dotenv
  ```
- API keys (for LLM Commander only):
  - A **Google Gemini API key** (`GOOGLE_API_KEY`)
  - Or an **OpenRouter API key** (`OPENROUTER_API_KEY`)
  - Place them in `/home/pi/.env`

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

### Full Digital Twin (Recommended)

Launch the GRBL driver, CNC URDF model, joystick driver, and teleop node all at once:
```bash
ros2 launch pi_bringup digital_twin.launch.py
```
Then open **RViz2** to visualize the CNC moving in real-time as you control it.

---

### Joystick CNC Control

The joystick teleop converts gamepad input into `/cmd_vel` messages, which the GRBL driver translates into real-time jog commands.

**Option A** — Use the joy launch file (starts both `joy_node` and `teleop_joy`):
```bash
ros2 launch pi_control joy.launch.py
```

**Option B** — Run nodes individually:

**Terminal 1** — Start the joystick driver:
```bash
ros2 run joy joy_node
```

**Terminal 2** — Start the teleop node:
```bash
ros2 run pi_control teleop_joy
```

> **Controls**: Hold **L1** (`buttons[4]`) as a deadman switch, then use the **left stick** to move the X/Y axes.

---

### LLM Agent Commander

Use natural language to command the CNC to draw shapes. The LLM agent (powered by Gemini) calculates coordinates and calls a `draw_shape` tool to move the plotter.

**1. Make sure the GRBL driver is running:**
```bash
ros2 run pi_firmware grbl_driver.py
```

**2. Start the LLM commander:**
```bash
ros2 run pi_control llm_commander
```

**3. Type commands in natural language:**
```
User: Draw a 20mm square in the center
Agent: ✅ Drawing square (5 points)...

User: Draw a circle with radius 15mm
Agent: ✅ Drawing circle (18 points)...

User: Draw a triangle at position 10,10 with side 30mm
Agent: ✅ Drawing triangle (4 points)...
```

The agent:
- Receives your natural language prompt
- Calculates the exact coordinates within the workspace bounds (X: 0–85mm, Y: 0–52mm)
- Calls the `draw_shape` tool, which publishes `Point` messages to `/cnc/move_to`
- The GRBL driver executes absolute positioning commands (`G90 G0`)

> **Safety**: The GRBL driver ignores LLM commands while the joystick is active.

---

### Serial Command Interface

Send raw G-code commands to the CNC controller via the `serial_command` topic.
```bash
ros2 run pi_firmware serial_command
```
```bash
# Example: Send a G-code command
ros2 topic pub /serial_command std_msgs/msg/String "data: 'G1 X1.0 Y1.0 F10'"
```

---

## Topic & Service Reference

### Topics

| Topic | Message Type | Direction | Description |
|-------|-------------|-----------|-------------|
| `/joy` | `sensor_msgs/Joy` | Pub | Joystick input from gamepad |
| `/cmd_vel` | `geometry_msgs/Twist` | Pub/Sub | Velocity commands (joystick → GRBL driver) |
| `/cnc/move_to` | `geometry_msgs/Point` | Pub/Sub | Absolute position commands (LLM → GRBL driver) |
| `/joint_states` | `sensor_msgs/JointState` | Pub | Live CNC axis positions from GRBL (`joint_x`, `joint_y`) |
| `/serial_command` | `std_msgs/String` | Sub | Raw G-code commands to CNC |
| `/led_control` | `std_msgs/Bool` | Sub | LED on/off control |
| `/temperature` | `example_interfaces/Float64` | Pub | CPU temperature readings |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/cnc/set_home` | `std_srvs/Trigger` | Reset GRBL position to (0, 0, 0) via `G92` |

---

## Hardware Configuration

### CNC Machine
This project is designed for a DIY 2-axis CNC plotter built with:
- **Controller**: Arduino Uno/Nano with GRBL firmware
- **Stepper Drivers**: A4988 motor drivers
- **Motors**: CD-ROM/DVD stepper motors (recycled from optical drives)
- **Axes**: X (0–85mm), Y (0–52mm)
- **Serial**: `/dev/ttyACM1` at 115200 baud

> **Note**: You may need to adjust the serial port path in `grbl_driver.py` or add udev rules for persistent device naming.

### Joystick Mapping

| Input | Action |
|-------|--------|
| **L1 button** (`buttons[4]`) | Deadman switch (hold to enable motion) |
| **Left stick X** (`axes[0]`) | CNC X-axis movement |
| **Left stick Y** (`axes[1]`) | CNC Y-axis movement |
| Deadzone | 0.05 (in `grbl_driver`) |

---

## Project Structure

```
ros-pi-cnc/
├── pi_bringup/
│   └── launch/
│       ├── bringup.launch.py         # Utility nodes (temp monitor, folder service)
│       └── digital_twin.launch.py    # Full stack: GRBL + URDF + Joystick
├── pi_control/
│   ├── launch/
│   │   └── joy.launch.py            # Joy node + teleop_joy
│   └── pi_control/
│       ├── teleop_joy.py            # Joystick → /cmd_vel
│       └── llm_commander.py         # LLM Agent → /cnc/move_to
├── pi_description/
│   ├── launch/
│   │   └── cnc.launch.py            # robot_state_publisher
│   └── urdf/
│       └── cnc.urdf.xacro           # CNC URDF (prismatic X/Y joints)
├── pi_firmware/
│   ├── firmware/                     # Arduino sketches (LED, stepper)
│   └── pi_firmware/
│       ├── grbl_driver.py           # GRBL serial driver + joint states
│       ├── joy_control_cnc.py       # Legacy direct-serial joystick control
│       ├── serial_command.py        # Raw G-code command interface
│       └── led_control.py           # LED control node
├── pi_interfaces/
│   └── srv/
│       └── CreateFolder.srv         # Custom service definition
└── pi_tests/
    └── pi_tests/
        ├── check_temperature.py     # CPU temp publisher
        ├── create_folder_server.py  # Folder creation service server
        └── create_folder_client.py  # Folder creation service client
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
