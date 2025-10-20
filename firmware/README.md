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

⚡ Serial Control Protocol

The system supports a concise, character-based serial protocol, primarily used by the Python IK controller for fast position updates. All commands must be terminated by a newline charactfer (`\n`).

Note: The firmware currently operates at 250000 baud for the interactive test.

| Command | Format | Description|
|---------|-------------------------------|----------------------------------------------------------------------------------------------------------|
| P       | `P<B> <S> <E> <W>\n` | Sets Base, Shoulder, Elbow, and Wrist angles (0-180 degrees). This is the primary IK movement command. |
| H       | `H\n`                          | Commands the arm to move to the predefined HOME position and automatically enables power (D2 HIGH).     |
| E       | `E\n`                          | Sets the D2 pin HIGH, applying torque/power to all motors.                                               |
| D       | `D\n`                          | Sets the D2 pin LOW, removing torque/power.                                                              |