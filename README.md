# 🤖 Arduino 4 DOF Acrylic Robot Arm Shield

This project features an Arduino shield for a simple **4 Degrees of Freedom (DOF)** Acrylic Robot Arm with a robotic gripper claw. 

---

## 📌 Pinout
Connect the Arduino shield as follows:

| Pin Number | Function          | Description                          |
|------------|-------------------|--------------------------------------|
| D9         | Base Servo        | Rotate the base 🌀                  |
| D10        | Shoulder Servo     | Move the shoulder 💪                 |
| D11        | Elbow Servo       | Bend the elbow 🤚                    |
| D3         | Gripper Servo     | Control the gripper claw ✊          |
| D2         | Servo Power Enable | Activate all servos ⚡               |

## 🚀 How to Run It
Use high-speed serial commands at **250000** baud to control the arm via a serial monitor.

### 📅 Basic Workflow:
1. Connect to the Arduino at **250000** baud.
2. Send **E** to power the servos.
3. Use **P** to position the arm.
4. Send **D** to disable servos when done.

### 📜 Commands:
- **🔋 Enable Servos:**
  - **Command**: E
  - **Action**: Powers on servos, allows movement.

- **✋ Disable Servos:**
  - **Command**: D
  - **Action**: Cuts power, servos go limp.

- **🎯 Go to Position:**
  - **Command**: P `<base>` `<shoulder>` `<elbow>` `<wrist>`
  - **Action**: Sets angles (0-180) for each servo. 
  - **Example**: P 90 45 120 180

- **🏠 Home Arm:**
  - **Command**: H
  - **Action**: Returns to home position (Base: 25°, Shoulder: 0°, Elbow: 60°, Wrist: 180°).

---

## 🖼️ Final Board
<img src="https://github.com/user-attachments/assets/94d61672-353b-4d17-881a-5043d9a84109" alt="IMG-20251025-WA0018" width="400" />
<img src="https://github.com/user-attachments/assets/8b99a545-8eb2-4781-97c2-95de9f968b7a" alt="IMG-20251025-WA0017" width="400" />
<img src="https://github.com/user-attachments/assets/f99d57ff-65df-4117-8323-c3f7e8e6f8e9" alt="IMG-20251025-WA0020" width="400" />

---

## 📜 License
This project is licensed under the **GNU General Public License (GPL) v3.0**. 
For details, view the full license [here](https://www.gnu.org/licenses/gpl-3.0.html).
