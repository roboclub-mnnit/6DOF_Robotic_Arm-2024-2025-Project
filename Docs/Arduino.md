# Arduino Firmware Documentation

This document provides detailed explanations of the Arduino sketches used to control the 6DOF robotic arm via servo motors.

---

## 📁 Code Structure

### 🔹 `Basecode`

- **Functionality**: Moves the robotic arm to a given position based on user input (X, Y, Z, Roll, Pitch, Yaw).
- **Use Case**: For basic forward positioning.

### 🔹 `CODE1`

- **Functionality**: Moves the robotic arm to a target position and returns to the original starting position.
- **Use Case**: Simple pick-and-return operations.

### 🔹 `CODE2`

- **Functionality**: Moves the robotic arm to pick up an object and returns to its original position.
- **Use Case**: Object pickup and repositioning tasks.

### 🔹 `FINAL`

- **Functionality**: Fully automated pick-and-place operation. The arm moves from home position, picks the object, places it elsewhere, and returns.
- **Use Case**: Industrial-like automation for repetitive tasks.

---

## 🔧 Libraries Used

- **Servo.h**: For controlling servo motors.
- **BasicLinearAlgebra.h (BLA)**: Used for matrix multiplication and transformations.

---

## ⚙️ Pin Configuration

| Servo Motor | Pin Number (Default) |
|-------------|----------------------|
| Base        | 3                    |
| Shoulder    | 4,5                  |
| Elbow       | 6                    |
| Wrist       | 9                    |
| Gripper     | 10                   |
| Rotator     | 11                   |

---

## 🌀 Operating Logic

1. Accept coordinates via Serial Monitor.
2. Compute required angles using inverse kinematics (in Python).
3. Pass angle values to servos.
4. Move arm to final position using the servo write command.
5. Optional: delay and return to home.

---


























