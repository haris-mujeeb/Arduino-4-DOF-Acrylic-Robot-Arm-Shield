# 4DOF Robotic Arm High-Speed Controller (PlatformIO Project)

This repository contains the highly optimized firmware for a 4DOF robotic arm. Built on PlatformIO, the primary design goal is maximum response speed and minimal servo jitter, achieved by bypassing the standard `Servo.h` library and **using direct hardware timer manipulation**.

## 🚀 Performance & Timers

The firmware utilizes the Arduino's native hardware timers (ATmega328P) to generate precise, stable 50Hz PWM signals.

| Joint    | Pin | Timer Configuration        | Note                          |
|----------|-----|----------------------------|-------------------------------|
| Base     | D9  | 16-bit (Timer1 OC1A)      | High Resolution               |
| Shoulder | D10 | 16-bit (Timer1 OC1B)      | High Resolution               |
| Elbow    | D11 | 8-bit (Timer2 OC2A)       | Standard Resolution           |
| Wrist    | D3  | 8-bit (Timer2 OC2B)       | Standard Resolution           |
| Enable   | D2  | Digital Output              | Master Power/Torque Control   |

The D2 pin is critical; it must be connected to an external relay or MOSFET/H-bridge to apply or remove power to all servos simultaneously.

## ⚙️ Firmware Architecture

The code is structured into two main components for abstraction and control:

| Component                | Description                                                                                                     |
|--------------------------|-----------------------------------------------------------------------------------------------------------------|
| `ArmController.cpp/h`     | Core Logic: Handles low-level PWM setup, configures Timer1/Timer2 registers, and includes the non-blocking goToPosition() method for instant angle updates. |
| `ArmTest.cpp/h`           | Debugging Utility: Implements the interactive serial interface for manual, step-by-step joint movement control. |
| `main.cpp`| Main Sketch: Performs all initialization, establishes the serial connection, and runs the continuous control loop (currently set to the interactive test mode). |

## ⚡ Serial Control Protocol

The system supports a concise, character-based serial protocol, primarily used by the Python IK controller for fast position updates. All commands must be terminated by a newline charactfer (`\n`).

Note: The firmware currently operates at 250000 baud for the interactive test.

| Command | Format | Description|
|---------|-------------------------------|----------------------------------------------------------------------------------------------------------|
| P       | `P<B> <S> <E> <W>\n` | Sets Base, Shoulder, Elbow, and Wrist angles (0-180 degrees). This is the primary IK movement command. |
| H       | `H\n`                          | Commands the arm to move to the predefined HOME position and automatically enables power (D2 HIGH).     |
| E       | `E\n`                          | Sets the D2 pin HIGH, applying torque/power to all motors.                                               |
| D       | `D\n`                          | Sets the D2 pin LOW, removing torque/power.                                                              |

## 🧠 Forward Kinematics & Transformation Model

The 4-DOF arm uses a **serial kinematic chain** consisting of:

1. Base rotation (Z-axis)
2. Shoulder pitch (Y-axis)
3. Elbow pitch (Y-axis)
4. Wrist/gripper (ignored for XYZ position)

To compute the 3D end-effector position from servo angles, the firmware applies **homogeneous transformation matrices**:

### Joint Angle Conversions

Servo angles are mapped to physical joint angles to account for mechanical offsets:

```python
\theta_1 = radians(Base_servo     - 90 - 28)                # Base rotation
\theta_2 = -radians(Shoulder_servo - 90 - 25)               # Shoulder pitch
\theta_q = -radians(Elbow_servo    - 25)                    # Elbow internal angle
\theta_3 = radians(90) - \theta_2 - \theta_q                            # External elbow angle
\phi  = \theta_2 + (\theta_3 - pi)                                    # Elbow link angle (absolute)
```

### Coordinate Frames

| Joint    | Axis | Motion                |
| -------- | ---- | --------------------- |
| Base     | Z    | Rotation              |
| Shoulder | Y    | Pitch (vertical lift) |
| Elbow    | Y    | Pitch (link bending)  |

Link lengths (arm geometry):

| Link   | Symbol      | Meaning          |
| ------ | ----------- | ---------------- |
| `L0_Z` | Base height | Base → shoulder  |
| `L1`   | Upper arm   | Shoulder → elbow |
| `L2`   | Forearm     | Elbow → wrist    |

---

### Homogeneous Transform Chain

The total end-effector transform:

$$
T_0^3 = T_0^1 \cdot T_1^2 \cdot T_2^3
$$

#### Base → Shoulder

Rotation about Z, translation up by `L0_Z`

$$
T_0^1=
\begin{bmatrix}
\cos \theta_1 & -\sin \theta_1 & 0 & 0 \\
\sin \theta_1 & \cos \theta_1 & 0 & 0 \\
0 & 0 & 1 & L_{0z} \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

#### Shoulder → Elbow

Rotation about Y, length `L1`

$$
T_1^2=
\begin{bmatrix}
\cos \theta_2 & 0 & \sin \theta_2 & L_1\cos \theta_2 \\
0 & 1 & 0 & 0 \\
-\sin \theta_2 & 0 & \cos \theta_2 & L_1\sin \theta_2 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

#### Elbow → Wrist

Rotation about Y, length `L2`

$$
T_2^3=
\begin{bmatrix}
\cos \phi & 0 & \sin \phi & L_2\cos \phi \\
0 & 1 & 0 & 0 \\
-\sin \phi & 0 & \cos \phi & L_2\sin \phi \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

### End-Effector Position

$$
P = T_0^3 \cdot 
\begin{bmatrix} 
0 \\ 
0 \\ 
0 \\ 
1
\end{bmatrix}
$$

This gives the arm's precise **(X, Y, Z)** position in 3D space, allowing:

* Smooth path planning
* Model-based IK
* External camera / AI control
* ROS / MoveIt compatibility

## 📐 Dimensions

Accurate measurements of the dimensions are essential for proper calculation of inverse and forward kinematics.

The dimensions of the kit used in the this project are given below.

![Dimensions](https://github.com/rahalnanayakkara/4dof-mearm-robot/blob/main/images/dimensions.jpg)

| Paramter | Value (mm) |
|:-------:|:---------:|
| $l_0$ | 0 |
| $h_1$ | 64 |
| $l_1$ | 15 |
| $l_2$ | 80 |
| $l_3$ | 80 |
|$l_{3I}$| 35 |
|$l_{3O}$| 35 |
| $l_4$ | 80 |
| $l_5$ | 65 |
| $d_5$ | 5 |

Before calculating configurations using inverse kinematics, the angles of the servo motors must be verified.
This can be done by setting all servos to $90^0$ and checking with the given diagram above.
