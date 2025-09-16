# Arduino 4 DOF Acrylic Robot Arm Shield
This project involves designing an Arduino shield for a simple 4 Degrees of Freedom (DOF) Acrylic Robot Arm equipped with a robotic gripper claw.

---

## Pinout
The Arduino shield connects to the Arduino board and provides the following pin configuration:

| Pin Number | Function          | Description                          |
|------------|-------------------|--------------------------------------|
| 2          | Servo 1 Control    | Controls the base rotation           |
| 3          | Servo 2 Control    | Controls the shoulder movement       |
| 4          | Servo 3 Control    | Controls the elbow movement          |
| 5          | Servo 4 Control    | Controls the gripper claw           |
| GND        | Ground             | Common ground connection             |
| VCC        | Power              | 5V power supply connection           |

---

## Schematic Diagram

![Schematic Diagram](/docs/schematic_diagram.png)

The schematic shows how the servos are connected to the Arduino through the shield, including power and ground connections.

---


## Block Diagram
![Block Diagram](/docs/block_diagram.jpg)

The block diagram illustrates the overall architecture of the robot arm system, showing the interaction between the Arduino, the shield, the servos, and the power supply.

## Working

1. **Power Supply**: The shield is powered by a 5V power supply connected to the VCC pin. Ensure that the power supply can handle the current requirements of all servos.

2. **Servo Control**: Each servo motor is controlled by a PWM signal sent from the Arduino. The pinout table above indicates which Arduino pins correspond to each servo.

3. **Arduino Code**: The Arduino code uses the Servo library to control the movement of the robot arm. Each servo is assigned to a specific pin, and the code defines the angles for movement.

4. **Robotic Gripper**: The gripper claw is controlled by the fourth servo, allowing it to open and close to grasp objects.

---

## License

This project is open source. Feel free to use, modify, and distribute it.
