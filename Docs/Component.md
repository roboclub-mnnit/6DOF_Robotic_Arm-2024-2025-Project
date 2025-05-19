# Hardware Components and Design Specifications

This document details all the mechanical and electronic components used in building the 6DOF robotic arm.

---

## 🔩 Mechanical Components

| Component          | Description                                      |
|-------------------|--------------------------------------------------|
| **3D Printed Links** | PLA/ABS printed arms and joints based on SolidWorks design |
| **Ball Bearings**     | Used to reduce friction and improve joint motion |
| **Gripper**           | Optional end-effector used to pick objects     |
| **Screws & Nuts**     | Fastening the servos and joints securely       |
| **Base Platform**     | Rigid support base for the entire arm          |

---

## ⚙️ Servo Motors

| Motor Model | Specs                          | Used For                   |
|-------------|----------------------------------|----------------------------|
| **MG995**   | 13 kg.cm @ 6V, ~60° in 0.2s      | General joint motion       |
| **MG958**   | 20+ kg.cm @ 6V, higher torque    | Base and shoulder joints   |

> All servos require external power using a stable **5V SMPS** or battery (not Arduino power).

---

## 💡 Electronics

| Component        | Function                              |
|------------------|----------------------------------------|
| **Arduino UNO**  | Main microcontroller for control logic |
| **Breadboard**   | Temporary connection of wires/pins     |
| **Jumper Wires** | Connect servos to Arduino              |
| **5V SMPS**      | Power supply for all servos (stable current) |

---

## 🛠️ Design Notes

- **CAD Design**: Created in SolidWorks. (Files available in `Solidworks File/`)
- **Joint Tolerances**: Adjusted for servo horn fitment.
- **Load Testing**: Rated for 500g payload with minimal deflection.
- **Cable Management**: Ensured clearance in simulation and physical build.

---

## 🧰 Assembly Tips

- Start from the base and move upwards while assembling.
- Test each servo independently before final assembly.
- Use thread-locking adhesive on metal joints for vibration resistance.
- Ensure the gripper does not exceed torque limits.

---
