# 🚜 Tomas Vinković PE-18 — ROS2 PS4 Teleop + Arduino Control

ROS2 sustav za upravljanje robota **Tomo (Tomas Vinković PE-18)** pomoću **PS4 DualShock kontrolera**, s Arduino mikrokontrolerom kao hardverskim interfejsom za motor, svjetla i blinkere.

Projekt omogućuje:
- 🎮 ručnu teleoperaciju preko PS4 kontrolera  
- 🤖 ROS2 integraciju (`cmd_vel`, state machine, sigurnosne provjere)  
- 🔌 serijsku komunikaciju s Arduinom  
- 💡 upravljanje svjetlima, pokazivačima i paljenjem motora  

Podržano:
- ROS2 **Jazzy / Humble / Iron**
- Fizički robot ili simulacija (Turtlesim)

---

## 📚 Table of Contents

- [📦 Struktura repozitorija](#-struktura-repozitorija)
- [🧠 Opis komponenti](#-opis-komponenti)
- [🎮 PS4 Kontroler – Mapiranje](#-ps4-kontroler--mapiranje)
- [🛠 Instalacija](#-instalacija)
- [▶️ Pokretanje](#️-pokretanje)
- [🧪 Simulacija](#-simulacija)
- [📡 Arduino](#-arduino)
- [🧩 ROS2 Topic Reference](#-ros2-topic-reference)
- [📌 Contributors](#-contributors)
- [📄 Licenca](#-licenca)

---

## 📦 Struktura repozitorija

```
tomo-hazarder/
├── arduino_serial/
│   └── arduino_serial.ino
├── control_tomo/
│   ├── ps4_controller.py
│   ├── ps4_teleop_node.py
│   └── arduino_serial_node.py
├── launch/
│   └── joy_ps4_teleop_launch.py
├── README.md
└── .gitignore
```

---

## 🧠 Opis komponenti

### PS4 Controller Parser
Python klasa za obradu PS4 `sensor_msgs/Joy` poruka.

### PS4 Teleop ROS2 Node
Glavni ROS2 teleoperacijski čvor.

### Arduino Serial ROS2 Node
ROS2 ↔ Arduino komunikacijski bridge.

### Arduino Firmware
Firmware za Arduino (Nano / Uno).

---

## 🎮 PS4 Kontroler – Mapiranje

| PS4 Input | Funkcija |
|---------|---------|
| Left Stick | Linearno i kutno kretanje |
| L1 | Omogućava kretanje |
| X (3s) | Arm / Disarm |
| O (3s) | Power Mode |
| Triangle | Start / Stop motora |
| Square (3s) | Light Mode |
| D-Pad | Upravljanje svjetlima |

---

## 🛠 Instalacija

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Fua6655/tomo-hazarder.git
cd ~/ros2_ws
colcon build --symlink-install
```

---

## ▶️ Pokretanje

```bash
ros2 run joy joy_node
ros2 launch control_tomo joy_ps4_teleop_launch.py
```

---

## 🧪 Simulacija

```bash
ros2 run turtlesim turtlesim_node
ros2 launch control_tomo joy_ps4_teleop_launch.py
```

---

## 📡 Arduino

Upload `arduino_serial.ino` koristeći Arduino IDE.

---

## 🧩 ROS2 Topic Reference

| Topic | Message |
|------|--------|
| /joy | sensor_msgs/Joy |
| /tomo/cmd_vel | geometry_msgs/Twist |
| /tomo/engine_start | std_msgs/Bool |
| /tomo/lights | std_msgs/UInt8MultiArray |

---

## 📌 Contributors

- Luka

---

## 📄 Licenca

MIT / Apache 2.0