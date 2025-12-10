# 🚜 Tomo Vinković PE-18 Control — ROS2 Jazzy

This package enables control of the **Tomo Vinković PE-18** tractor robot (or Turtlesim simulation) using a **PS4 DualShock controller** via the `joy` ROS2 driver.

Designed for **ROS2 Jazzy** on Ubuntu.

---

## 📦 Package Structure

control_tomo/
├── launch/
│ └── joy_ps4_teleop_launch.py
├── control_tomo/
│ ├── init.py
│ ├── ps4_controller.py
│ ├── ps4_teleop_node.py
├── images/
│ └── tomo_pe18.jpg
├── setup.py
├── package.xml
└── README.md
└── .gitignore

---

## 🕹 Controller Mapping (PS4)

| Input | Action |
|------|--------|
| Left Stick | Movement: linear + angular |
| L1 | Movement enable (must be held) |
| D-Pad | Speed mode: High / Low |
| X button (hold 3s) | Power ON/OFF safety toggle |

Debug logs are printed to the terminal.

---

## 🐢 Simulation Support (Turtlesim)

The launch file includes:
- `joy_node`
- `tomo_control_node`
- `turtlesim_node`

So you can validate control logic visually.

---

## ▶️ Quick Start

### 1️⃣ Install dependencies
```bash
sudo apt install ros-jazzy-joy ros-jazzy-turtlesim
```

### 2️⃣ Build your workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
### 3️⃣ Launch!
```bash
ros2 launch control_tomo joy_ps4_teleop.launch.py

ros2 launch control_tomo turtlesim.launch.py
```
## 🔧 Manual Testing

Only joystick:
```bash
ros2 run joy joy_node
```
Only turtlesim:
```bash
ros2 run turtlesim turtlesim_node
```
Only Tomo Control:
```bash
ros2 run control_tomo control_Node
```
## 📸 Robot Image

tomo-hazarder/images/tomo_pe18.jpg

## 🧠 Credits

    ROS2 integration and control logic by Luka
