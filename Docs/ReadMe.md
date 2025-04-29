# 6-DOF Robotic Arm Project

This repository contains code and documentation for a 6-Degree of Freedom (6DOF) robotic arm built using MG995/MG958 servo motors and Arduino UNO. The project implements inverse kinematics and forward kinematics for precise control of the arm's end-effector position in 3D space.

---

## 📑 Table of Contents

- Project Overview
- Repository Structure
- Hardware Requirements
- Software Requirements
- Installation and Setup
- Usage
- Coordinate System and Offsets
- Kinematics and Transformation Matrices
- Angle Ranges
- Photo Gallery
---

## 📌 Project Overview

This project demonstrates the design and development of a 6DOF robotic manipulator for industrial automation tasks. It offers ±2mm positioning accuracy with a 0.5 kg payload capacity. The arm is controlled via Arduino using trajectory planning based on inverse kinematics and includes simulation in Python.

---

## Repository Structure


- `Arduino Files/`

  - `Basecode`: Firmware for  move robotic arm to given position.
  - `CODE1`: Firmware for move the robotic arm to the given postion and return in its original position.
  - `CODE2`: Firmware for move the robotic arm to given postion and pick a object and return in its original position.
  - `FINAL`: Firmware for move robotic arm from its original postion pick object from a position and place it on another position and move back to it original position.

- `Python File/`

  - `kinematics/`
    - `Inverse Kinematics.py`: Script to compute six joint angles of the robotic arm.
  - `simulation/`
    - `Simulation.py`: 3D simulation of all joint of the Robotic arm.
    

- `Solidworks File/`

  - (Placeholder for SolidWorks design files, e.g., `.sldprt` or `.sldasm` files)

- `docs/`

  - `ReadMe.md`: Main project overview and instructions.
  - `arduino.md`: Detailed documentation for Arduino firmware.
  - `components.md`: Information on design specifications and hardware components.
  - `python.md`: Detailed documentation for Python simulation and kinematics scripts.

## 🔧 Hardware Components

- Arduino UNO
- MG995 / MG958 Servo Motors
- Jumper Wires & Breadboard
- 3D Printed Structural Parts
- SMPS Power supply (5V)
- Ball Bearings
- Gripper (optional)

---

## 💻 Software Stack

- Arduino IDE
- BLA Library (for multiplication of mtrix in Arduino IDE)
- Python (with Matplotlib, NumPy)
- SolidWorks (for modeling)
- Inverse Kinematics (custom script)
- Forward Kinematics (DH Parameter based)

---

## 📐 Kinematics and Control

We use Denavit-Hartenberg (D-H) parameters to derive transformation matrices for forward kinematics and solve inverse kinematics numerically for joint angle calculations. The control flow is as follows:

1. **Input Desired (x, y, z)**
2. **Calculate wrist center from gripper center coordinate**
3. **Calculate first three Joint Angles via IK**
4. **Find R3_6 matrix for last three joints angle**
5. **Write Python code for simulation**
6. **Move Servo Motors via Arduino**

---

## ⚙️ Installation & Setup

1. **Python Setup**:

   - Install Python 3.x.

   - Install libraries:

     ```bash
     pip install numpy matplotlib
     ```

2. **Arduino Setup**:

   - Install Arduino IDE from arduino.cc.
   - Connect servos to Arduino pins (default: 3,5,6, 9, 10, 11).
   - Upload the appropriate `.ino` file for each leg.

3. **Simulation**:

   - Run Python scripts for visualization.
   - Adjust joint angles using sliders.

4. **Physical Robot**:

   - Power servos and connect to Arduino.
   - Use Serial Monitor (9600 baud) to input coordinates.


## Usage

### Simulation

- Run `Simulation.py` for all joint angles:

  ```bash
  python Simulation.py
  ```

  - Adjust X, Y, Z,ROLL,PITCH ,YAW values via sliders.
  - Use XY, XZ, YZ buttons to change views.
  - View end effector coordinates on the plot.



### Inverse Kinematics

- Run `Inverse Kinematics.py`:

  ```bash
  python Inverse Kinematics.py
  ```

  - Input End Effector coordinates (x, y, z) and orientation of the End Effector (ROLL,PITCH,YAW).
  - Outputs theta1, theta2, theta3, theta4, theta5, theta6 in degrees.

### Arduino Control

- Upload `.ino` file for each leg.
- Open Serial Monitor (9600 baud).
- Enter x, y, z  coordinate and ROLL,PITCH,YAW values to move servos.


## Kinematics and Transformation Matrices

Link Length: `a1 = 5.38 cm`, `a2 = 0.0 cm`, `a3 = 11.217 cm`, `a4 = 12.876 cm`, `a5 = 4.405 cm`.


### Rotation Matrices

```python
R0_1 = np.array([[-np.sin(T1), 0, np.cos(T1)],
                 [np.cos(T1),  0, np.sin(T1)],
                 [0,           1,          0]])

R1_2 = np.array([[np.cos(T2), -np.sin(T2), 0],
                 [np.sin(T2),  np.cos(T2), 0],
                 [0,           0,          1]])

R2_3 = np.array([[1, 0, 0],
                 [0, 0, 1],
                 [0, -1, 0]])
```

### DH Parameter matrix 

```python
def DH_Params_H_Matrix(i,j,q1=0,q2=0,q3=0,q4=0,q5=0,q6=0):
    DH_params=np.array([
        [q1,np.pi/2,a2,a1],
        [q2,0,a3,0],
        [q3+np.pi/2,np.pi/2,0,0],
        [q4,-np.pi/2,0,a4],
        [q5,np.pi/2,0,0],
        [q6,0,0,a5]
    ])
    H_matrix= Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]                                                                                   
    ])
    for k in range(i,j):                                                                                                                             
        H2_matrix=Matrix([                                     
            [np.cos(DH_params[k,0]),-np.sin(DH_params[k,0])*np.cos(DH_params[k,1]),np.sin(DH_params[k,0])*np.sin(DH_params[k,1]),DH_params[k,2]*np.cos(DH_params[k,0])],
            [np.sin(DH_params[k,0]),np.cos(DH_params[k,1])*np.cos(DH_params[k,0]),-np.cos(DH_params[k,0])*np.sin(DH_params[k,1]),DH_params[k,2]*np.sin(DH_params[k,0])],
            [0,np.sin(DH_params[k,1]),np.cos(DH_params[k,1]),DH_params[k,3]],
            [0,0,0,1]
        ])
        H_matrix=np.dot(H_matrix,H2_matrix)         
    return np.matrix(H_matrix)
```

### Rotation Matrix

```python
def get_Joint_Variables(finalX, finalY, finalZ, roll, pitch, yaw):
    R0_6_initial = np.array([
        [0, 0, 1],
        [0, -1, 0],
        [1, 0, 0]
    ])

    R_6_rotation = np.array([
        [np.cos(pitch)*np.cos(roll), np.sin(pitch)*np.sin(yaw)-np.sin(roll)*np.cos(pitch)*np.cos(yaw), np.sin(pitch)*np.cos(yaw)+np.sin(roll)*np.sin(yaw)*np.cos(pitch)],
        [np.sin(roll),               np.cos(roll)*np.cos(yaw),                                         -np.sin(yaw)*np.cos(roll)],
        [-np.sin(pitch)*np.cos(roll),np.sin(pitch)*np.sin(roll)*np.cos(yaw)+np.sin(yaw)*np.cos(pitch), -np.sin(pitch)*np.sin(roll)*np.sin(yaw)+np.cos(pitch)*np.cos(yaw)]
    ])

    R0_6 = np.dot(R0_6_initial, R_6_rotation)
    print(R0_6)

```

## Photo Gallery

- Manipulator Diagram:
  ![Manipulator diagram (2)](../Image/Manipulater%20diagram.jpeg)

- Robotic Dog(Model):
  
  ![Solidworks design of the 6DOFRobotic arm](../Image/Solidworks%20design.png)
- Robotic Dog(Actual Bot):
  ![3D Printed Robotic arm](../Image/3D%20printed%20robotic%20arm.jpg)



















