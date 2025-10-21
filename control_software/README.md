# 4DOF Inverse Kinematics Serial Controller (Python)

This Python script is a Cartesian control layer for the 4DOF robotic arm, allowing you to move the end-effector by specifying (X, Y, Z) coordinates instead of raw joint angles. It communicates directly with the Arduino firmware using the established serial protocol.

## 🛠️ Prerequisites

You must have the pyserial library installed to enable communication with the Arduino.

`pip install pyserial`

## 📝 Configuration

Before running the script, you must configure the following variables in ik_serial_controller.py to match your hardware and connection settings.

1. Robot Physical Parameters (Link Lengths)

Ensure these match your physical arm's geometry. All measurements are in millimeters (mm).

| Parameter | Default Value | Description                                                    |
|-----------|---------------|----------------------------------------------------------------|
| L0_Z      | 150.0        | Z-height of the Shoulder joint (J2) above the Base plane (J1).|
| L1        | 150.0        | Length of the Shoulder-to-Elbow link.                         |
| L2        | 120.0        | Length of the Elbow-to-Wrist link.                           |

## Length of the Elbow-to-Wrist link.

2. Serial Settings

| Parameter      | Default Value              | Description                                                                                          |
|----------------|----------------------------|------------------------------------------------------------------------------------------------------|
| SERIAL_PORT    | 'COM3'                     | The port assigned to your Arduino (e.g., COM3 on Windows, /dev/ttyACM0 on Linux, /dev/tty.usbmodemXXXX on macOS). |
| BAUD_RATE      | 115200                     | Must match the Serial.begin() speed in your Arduino sketch.                                          |

## 🤖 Inverse Kinematics (IK) Assumptions

The ArmIK class uses a 3-link planar IK solver (Base, Shoulder, Elbow, Wrist) with the following assumptions:

Planar Linkage: The Shoulder, Elbow, and Wrist operate in a 2D plane defined by the Base rotation (Theta 1).

Elbow-Up Solution: The current solver is configured to always find the "elbow-up" solution (the most common configuration for benchtop arms).

Wrist Orientation: The optional fourth parameter, wrist_orient_deg, allows you to define the final angle of the wrist/gripper relative to the horizontal.

## ▶️ How to Run

1. Upload the Arduino firmware
2. Close the Arduino Serial Monitor.
3. Execute the Python script:
`python ik_serial_controller.py`

## Usage

When prompted, enter the target coordinates separated by spaces:
`Target (X Y Z [W_Orient_Deg]):`


Example: `150 0 100 45` (X=150, Y=0, Z=100, Wrist=45°).

The script solves the IK, checks reach limits, and sends the P<B> <S> <E> <W>\n command to the Arduino.