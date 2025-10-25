# Robotic Arm Arduino Shield

This directory contains the KiCad hardware design files for the custom 4-DOF (Degrees of Freedom) robotic arm controller shield.

## Features

-   **Direct Arduino Uno Integration:** The shield is designed to plug directly onto an Arduino Uno.
-   **4-Servo Control:** Provides dedicated headers for four servo motors (Base, Shoulder, Elbow, Wrist).
-   **Stable PWM:** The design relies on the Arduino's hardware timers (Timer1 and Timer2) to generate a stable and precise PWM signal, which is crucial for jitter-free servo operation. The pinout is specifically chosen to match these timers.
-   **Power Control:** Includes a MOSFET circuit to enable or disable power to the servos, allowing the arm to go "limp" without being physically disconnected. This is controlled by digital pin D2.

## Pinout

The shield uses the following Arduino pins, which are hard-coded in the firmware to align with the hardware timers:

-   **D9:** Base Servo (Timer1 OC1A)
-   **D10:** Shoulder Servo (Timer1 OC1B)
-   **D11:** Elbow Servo (Timer2 OC2A)
-   **D3:** Wrist Servo (Timer2 OC2B)
-   **D2:** Servo Power Enable (MOSFET Gate)

## Final PCB

Here is an image of the assembled PCB:

<img src="https://github.com/user-attachments/assets/94d61672-353b-4d17-881a-5043d9a84109" alt="Assembled Robotic Arm Shield" width="400" />

The KiCad project files (`.kicad_prl`, `.kicad_pcb`, `.kicad_sch`) and manufacturing outputs (Gerbers, PDFs) are located in the `kicad_files` subdirectory.
