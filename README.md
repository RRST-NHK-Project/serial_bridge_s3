# **serial_bridge**

## 1. Overview

`serial_bridge` is a ROS 2 package that provides a bidirectional bridge between ROS 2 nodes and microcontrollers via serial communication.  
It converts ROS 2 messages into custom binary serial frames and parses received frames back into ROS 2 messages.

This package is designed for real-time robot hardware control, such as dc-motors, rc-servo, encoders, and general-purpose I/O, and is actively used in the NHK Project, RRST, at Ritsumeikan University.

---

## 2. System Requirements

| Item | Description |
|:---|:---|
| OS | Ubuntu 24.04 LTS |
| ROS | ROS 2 Jazzy |
| Hardware | PC connected to MCU via USB or UART |

> **Note**:  
> Stable communication requires proper serial permissions.  
> Ensure the user belongs to the `dialout` group when using `/dev/ttyUSB*` or `/dev/ttyACM*`.

---

## 3. Features

- ROS 2 <-> Microcontroller serial communication
- Custom binary frame protocol using `int16` data arrays
- Low-latency, lightweight message transfer
- Checksum-based frame validation
- ESP-based microcontroller implementation provided
- Protocol and architecture designed to be portable across MCU platforms

---

## 4. Communication Architecture

```
ROS 2 Node (serial_bridge)
 ├─ Subscribe: std_msgs/msg/Int16MultiArray
 ├─ Publish  : std_msgs/msg/Int16MultiArray
 │
 └─ Serial Port (/dev/ttyUSB*, /dev/ttyACM*)
        │
        └─ Microcontroller
            ├─ DC-Motor control
            ├─ RC-Servo control
            ├─ Encoder reading
            └─ General-purpose I/O
```

---

## 5. Frame Format

### 5.1 PC → Microcontroller (TX)

```
[0] START_BYTE   : 0xAA
[1] DEVICE_ID
[2] LENGTH       : Payload length (bytes)
[3] DATA[0]
...
[n] DATA[n]
[n+1] CHECKSUM
```

- `DATA` is an array of `int16_t`
- `DEVICE_ID` is a unique identifier assigned to each microcontroller
- `CHECKSUM` is calculated using XOR of all previous bytes

---

### 5.2 Microcontroller → PC (RX)

- Same structure as TX
- Typically used for encoder values, sensor data, and status feedback

---

## 6. ROS 2 Interfaces

### 6.1 Subscribed Topics (ROS → MCU)

| Topic | Type | Description |
|:---|:---|:---|
| `/serial_tx_[DEVICE_ID]` | `std_msgs/msg/Int16MultiArray` | Control commands and parameters |

---

### 6.2 Published Topics (MCU → ROS)

| Topic | Type | Description |
|:---|:---|:---|
| `/serial_rx_[DEVICE_ID]` | `std_msgs/msg/Int16MultiArray` | Encoder values, sensor data, debug information |

---

## 7. Getting Started

### 7.1 Clone the Repository

```
cd ~/ros2_ws/src
git clone https://github.com/RRST-NHK-Project/serial_bridge.git
```

---

### 7.2 Build

```
cd ~/ros2_ws
colcon build --packages-select serial_bridge
source install/setup.bash
```

---

### 7.3 Run

```
ros2 run serial_bridge serial_bridge_node
```

Serial port settings such as device name and baud rate are configured in the source code or via ROS 2 parameters, depending on the implementation.

---

## 8. Directory Structure

| Path | Description |
|:---|:---|
| `/esp32_serial_bridge` | Reference implementation for ESP32-based microcontrollers (PlatformIO project) |
| `/src` | Source code for the ROS 2 node |

---


## 10. Credits
Developed by NHK Project, RRST, Ritsumeikan University, Japan (2024)
- Official Website: https://www.rrst.jp
- X (Twitter): https://x.com/RRST_BKC

![Logo](https://www.rrst.jp/img/logo.png)
