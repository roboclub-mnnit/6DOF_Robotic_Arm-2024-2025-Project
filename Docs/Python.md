# Python Simulation and Kinematics Documentation

This file describes the Python scripts used to simulate and calculate the kinematics of the 6DOF robotic arm.

---

## 📂 Directory Structure

- `kinematics/`
  - `Inverse Kinematics.py`: Calculates the six joint angles using numerical inverse kinematics.
- `simulation/`
  - `4 Legs Simulation.py`: Generates a 3D simulation of the robotic arm’s motion using matplotlib.

---

## 🧠 Kinematics Theory

### ➤ Inverse Kinematics

The script receives:
- Target coordinates: (X, Y, Z)
- Orientation: (Roll, Pitch, Yaw)

And outputs:
- Joint angles: θ1 to θ6

It breaks down the solution in two stages:
1. **First Three Joints**: Uses geometric IK for shoulder, elbow, and base.
2. **Last Three Joints**: Uses rotation matrices to compute wrist orientation.

---

## 🧩 Libraries Used

- **NumPy**: For matrix computation
- **Matplotlib**: For 3D simulation and visualization
- **SymPy** *(optional)*: For symbolic matrix generation (used in DH transformation)

---

## 🖼️ Simulation Features

- Real-time plotting of joint configurations.
- Interactive sliders to change:
  - X, Y, Z (position)
  - Roll, Pitch, Yaw (orientation)
- XY, YZ, XZ view toggles.
- Plots the full kinematic chain of the arm.

---

## 🚀 Running the Scripts

### 1. Install Dependencies

```bash
pip install numpy matplotlib
