import serial
import time
import math
import sys
# --- NEW: SERIAL PORT DISCOVERY ---
try:
    import serial.tools.list_ports
except ImportError:
    print("Warning: serial.tools.list_ports not found. Port selection disabled.")
    serial_ports = None

# --- VIZUALIZATION IMPORTS ---
# Requires matplotlib: pip install matplotlib
try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
except ImportError:
    print("Warning: Matplotlib not found. Visualization will be disabled.")
    plt = None


# --- 1. ROBOT PHYSICAL PARAMETERS (MUST BE ADJUSTED) ---
# Define your link lengths in millimeters (mm)
# Measure these precisely on your Keyestudio Arm from joint-center to joint-center.
L0_Z = 55.0  # Z-height of the shoulder joint (J2) above the base plane (J1)
L1 = 80.0    # Length of the shoulder-to-elbow link (J2 to J3)
L2 = 80.0    # Length of the elbow-to-wrist link (J3 to End-Effector)

# Maximum theoretical reach of the arm
MAX_REACH = L1 + L2

# --- 2. SERIAL COMMUNICATION SETUP ---
# SERIAL_PORT is now determined dynamically by the select_serial_port function
BAUD_RATE = 250000     # Match the rate set in your Arduino code

# --- 3. ANGLE/LIMIT UTILITIES ---

# Default angles for testing (Base, Shoulder, Elbow, Gripper)
DEFAULT_HOME_ANGLES = [90, 90, 90, 90] 

def select_serial_port():
    """Lists available serial ports and prompts the user for selection."""
    if 'serial_tools' not in sys.modules:
        return 'COM10' # Fallback to hardcoded port if discovery tool is missing

    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        print("No serial ports found. Please check connections.")
        return None

    print("\n--- Available Serial Ports ---")
    for i, port in enumerate(ports):
        # Display index, device name, and descriptive info
        print(f"[{i + 1}]: {port.device} ({port.description})")
    print("------------------------------")

    while True:
        try:
            selection = input("Select the Arduino port number (e.g., 1, 2, etc.): ")
            if selection.lower() == 'quit':
                return None
            
            index = int(selection) - 1
            if 0 <= index < len(ports):
                return ports[index].device
            else:
                print("Invalid selection. Please enter a number from the list.")
        except ValueError:
            print("Invalid input. Please enter a number or 'quit'.")

class ArmIK:
    def __init__(self, L0_Z, L1, L2):
        self.L0_Z = L0_Z
        self.L1 = L1
        self.L2 = L2
        # Initialize visualization plot objects
        if plt:
            self.fig = plt.figure(figsize=(8, 8))
            self.ax = self.fig.add_subplot(111, projection='3d')
            plt.ion() # Turn on interactive mode for live updates
            self.ax.set_title("4DOF Arm Kinematic Model")
            self._setup_plot_limits()
            
    def _setup_plot_limits(self):
        """Sets consistent axis limits based on max reach."""
        plot_limit = MAX_REACH + L0_Z + 20 # Add buffer
        self.ax.set_xlim([-plot_limit, plot_limit])
        self.ax.set_ylim([-plot_limit, plot_limit])
        self.ax.set_zlim([0, plot_limit])
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.view_init(elev=30, azim=-60) # Set initial view angle

    def clamp(self, value, min_val, max_val):
        """Clamps a value within a specified range."""
        return max(min_val, min(max_val, value))

    def solve_ik(self, x, y, z, grip_angle_deg=90):
        """
        Calculates the four servo angles (Base, Shoulder, Elbow, Gripper) for a target (x, y, z).
        
        Args:
            x, y, z (float): Target Cartesian coordinates (mm).
            grip_angle_deg (float): Desired angle for the gripper (J4), e.g., 0=closed, 180=open.
            
        Returns:
            list: [Base, Shoulder, Elbow, Gripper] servo angles (0-180 range), or None on failure.
        """
        
        # --- J1: Base Angle (Theta 1 - Yaw) ---
        if x == 0 and y == 0:
            angle_base = 0.0 # Straight forward
        else:
            angle_base_rad = math.atan2(y, x)
            angle_base = math.degrees(angle_base_rad)
        
        servo_base = int(self.clamp(angle_base + 90.0, 0, 180))

        # --- Project to 2D Plane (R-Z) ---
        R = math.sqrt(x**2 + y**2) # Horizontal reach from the base
        Z_eff = z - self.L0_Z      # Z height relative to the shoulder joint (J2)

        D_sq = R**2 + Z_eff**2
        D = math.sqrt(D_sq)

        if D > MAX_REACH or D < abs(self.L1 - self.L2) or R < 0:
            # print(f"IK ERROR: Target ({R:.1f}, {Z_eff:.1f}) out of reach (D={D:.1f}). Max: {MAX_REACH:.1f}")
            return None

        # --- J3: Elbow Angle (Theta 3) - Pitch ---
        try:
            cos_alpha_L2 = (self.L1**2 + self.L2**2 - D_sq) / (2 * self.L1 * self.L2)
            cos_alpha_L2 = self.clamp(cos_alpha_L2, -1.0, 1.0) 
            alpha_L2_rad = math.acos(cos_alpha_L2)
        except ZeroDivisionError:
            print("IK ERROR: Division by zero during elbow calculation.")
            return None

        angle_elbow_rad = math.pi - alpha_L2_rad
        angle_elbow_deg = math.degrees(angle_elbow_rad)
        
        # Servo Mapping J3: This is highly specific. 
        servo_elbow = int(self.clamp(angle_elbow_deg, 0, 180))


        # --- J2: Shoulder Angle (Theta 2) - Pitch ---
        beta_rad = math.atan2(Z_eff, R) 
        
        cos_alpha_L1 = (D_sq + self.L1**2 - self.L2**2) / (2 * D * self.L1)
        cos_alpha_L1 = self.clamp(cos_alpha_L1, -1.0, 1.0) 
        alpha_L1_rad = math.acos(cos_alpha_L1)
        
        angle_shoulder_rad = beta_rad + alpha_L1_rad
        angle_shoulder_deg = math.degrees(angle_shoulder_rad)

        # Servo Mapping J2: Needs tuning.
        servo_shoulder = int(self.clamp(angle_shoulder_deg, 0, 180))
        
        # --- J4: Gripper Control ---
        servo_gripper = int(self.clamp(grip_angle_deg, 0, 180))

        # Assemble results: [Base, Shoulder, Elbow, Gripper]
        angles = [servo_base, servo_shoulder, servo_elbow, servo_gripper]
        
        # print(f"\nIK Solved: Base={angles[0]} | Shoulder={angles[1]} | Elbow={angles[2]} | Gripper={angles[3]}")
        
        return angles

    def forward_kinematics(self, servo_angles):
        """
        Calculates the joint coordinates (X, Y, Z) from the servo angles.
        """
        [B_servo, S_servo, E_servo, G_servo] = servo_angles
        
        # Reverse Servo Mappings to get Joint Angles (in radians)
        # J1 (Base)
        theta1 = math.radians(B_servo - 90.0)
        # J2 (Shoulder)
        theta2 = math.radians(S_servo)
        # J3 (Elbow) - Assuming 180 is straight, 0 is fully bent/folded
        theta3 = math.radians(E_servo)
        
        # 1. Base (P0)
        P0 = [0, 0, 0]
        
        # 2. Shoulder Joint (P1 = J2)
        P1 = [0, 0, self.L0_Z]
        
        # 3. Elbow Joint (P2 = J3) - Position based on P1 and L1 rotated by theta1, theta2
        R1 = self.L1 * math.cos(theta2)
        Z1 = self.L1 * math.sin(theta2)
        
        P2_x = R1 * math.cos(theta1)
        P2_y = R1 * math.sin(theta1)
        P2_z = self.L0_Z + Z1
        P2 = [P2_x, P2_y, P2_z]
        
        # 4. Wrist/End-Effector (P3) - Position based on P2 and L2 rotated by theta1, (theta2 + theta3)
        phi = theta2 + theta3
        R2 = self.L2 * math.cos(phi)
        Z2 = self.L2 * math.sin(phi)
        
        P3_x = P2_x + R2 * math.cos(theta1)
        P3_y = P2_y + R2 * math.sin(theta1)
        P3_z = P2_z + Z2
        P3 = [P3_x, P3_y, P3_z]
        
        return [P0, P1, P2, P3]

    def visualize_arm(self, joint_coords, target_x=None, target_y=None, target_z=None, title=None):
        """Plots the robot arm links in a 3D matplotlib window."""
        if not plt:
            return

        self.ax.clear()
        self._setup_plot_limits()
        
        # Unpack coordinates
        P0, P1, P2, P3 = joint_coords
        
        # Plot Links
        x_coords = [P0[0], P1[0], P2[0], P3[0]]
        y_coords = [P0[1], P1[1], P2[1], P3[1]]
        z_coords = [P0[2], P1[2], P2[2], P3[2]]
        
        self.ax.plot(x_coords[1:], y_coords[1:], z_coords[1:], 
                     'b-', linewidth=5, label='Links')
        
        self.ax.scatter(x_coords[1:], y_coords[1:], z_coords[1:], 
                        c=['r', 'g', 'm'], marker='o', s=100, label='Joints')

        # Plot Target if provided
        if target_x is not None:
            self.ax.scatter([target_x], [target_y], [target_z], 
                            c='k', marker='x', s=150, label='Target')
            current_title = f"Target: X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}"
        elif title:
            current_title = title
        else:
            current_title = "Arm Kinematic Model"
            
        self.ax.set_title(current_title)
        
        plt.draw()
        plt.pause(0.001)


def send_command(ser, angles):
    """Formats and sends the angle command over serial."""
    command_str = f"P{angles[0]} {angles[1]} {angles[2]} {angles[3]}\n"
    ser.write(command_str.encode())
    # print(f"Sent: {command_str.strip()}")
    time.sleep(0.05) # Small delay for the servo/Arduino to process


# --- 4. TEST FUNCTIONS ---

def test_joint_sweep(ik_solver, ser):
    """Tests each motor by sweeping it from 0 to 180 degrees while others stay at home."""
    print("\n--- Running Joint Sweep Test (Motor Calibration) ---")
    
    # Servo Index mapping for clarity
    JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Gripper"]
    current_angles = list(DEFAULT_HOME_ANGLES)

    for joint_index, joint_name in enumerate(JOINT_NAMES):
        print(f"\nTesting Joint: {joint_name} (Index: {joint_index})")
        
        # Sweep forward (0 to 180)
        for angle in range(0, 181, 10):
            angles_to_send = list(current_angles) # Start with current angles
            angles_to_send[joint_index] = angle   # Change only the joint being tested
            
            # Send command
            send_command(ser, angles_to_send)
            
            # Visualize
            joint_coords = ik_solver.forward_kinematics(angles_to_send)
            ik_solver.visualize_arm(joint_coords, title=f"Joint Sweep: {joint_name} @ {angle}°")

        # Sweep backward (180 to 0)
        for angle in range(180, -1, -10):
            angles_to_send = list(current_angles)
            angles_to_send[joint_index] = angle
            
            send_command(ser, angles_to_send)
            
            joint_coords = ik_solver.forward_kinematics(angles_to_send)
            ik_solver.visualize_arm(joint_coords, title=f"Joint Sweep: {joint_name} @ {angle}°")
            
        # Reset the tested joint back to the home angle before moving to the next joint
        current_angles[joint_index] = DEFAULT_HOME_ANGLES[joint_index]
        send_command(ser, current_angles)
    
    print("\nJoint Sweep Test Complete.")

def test_linear_motion(ik_solver, ser):
    """Commands the arm to move the end-effector in a straight line in 3D space."""
    print("\n--- Running Linear Motion Test (IK Validation) ---")
    
    # 1. Define Start and End Points for the line segment (in mm)
    
    # Example 1: Forward-Upward Motion (a safe path)
    X_start, Y_start, Z_start = 100.0, 0.0, 40.0
    X_end, Y_end, Z_end = 150.0, 50.0, 100.0
    
    # Example 2: More complex arc (uncomment to try)
    # X_start, Y_start, Z_start = 120.0, -20.0, 80.0
    # X_end, Y_end, Z_end = 50.0, 100.0, 150.0 
    
    num_steps = 50
    print(f"Moving from Start: ({X_start}, {Y_start}, {Z_start}) to End: ({X_end}, {Y_end}, {Z_end}) in {num_steps} steps.")

    for i in range(num_steps + 1):
        t = i / num_steps # Linear interpolation parameter (0.0 to 1.0)
        
        # Calculate interpolated Cartesian coordinates
        x = X_start + t * (X_end - X_start)
        y = Y_start + t * (Y_end - Y_start)
        z = Z_start + t * (Z_end - Z_start)
        
        # Solve IK for the intermediate point
        angles = ik_solver.solve_ik(x, y, z, grip_angle_deg=90)
        
        if angles:
            # Send command and visualize the movement
            send_command(ser, angles)
            
            # The target (x, y, z) is the theoretical path, the visualization shows the result
            joint_coords = ik_solver.forward_kinematics(angles)
            ik_solver.visualize_arm(joint_coords, x, y, z, title=f"Linear Motion Test: Step {i}/{num_steps}")
        else:
            print(f"Warning: Step {i} ({x:.1f}, {y:.1f}, {z:.1f}) is unreachable. Stopping.")
            break

    print("\nLinear Motion Test Complete.")
    
def interactive_ik_mode(ik_solver, ser):
    """The interactive mode for entering custom coordinates."""
    print("\n--- Running Interactive IK Mode ---")
    
    while True:
        # Check for serial feedback from Arduino (e.g., "ACK" or "ERR")
        if ser.in_waiting > 0:
            feedback = ser.readline().decode().strip()
            if feedback:
                print(f"[Arduino]: {feedback}")

        user_input = input("Target (X Y Z [Grip_Deg]) or 'back': ")
        
        if user_input.lower() in ['quit', 'exit', 'back']:
            break

        parts = user_input.split()
        
        if not (3 <= len(parts) <= 4):
            print("Invalid input format. Use: X Y Z [Grip_Deg]")
            continue
            
        try:
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2])
            grip_angle = float(parts[3]) if len(parts) == 4 else 90.0 # Default grip angle
        except ValueError:
            print("Coordinates/Angle must be valid numbers.")
            continue

        # --- Solve IK ---
        angles = ik_solver.solve_ik(x, y, z, grip_angle)
        print(f"IK Solved: Base={angles[0]} | Shoulder={angles[1]} | Elbow={angles[2]} | Gripper={angles[3]}")
        
        if angles:
            # 1. Visualize the arm using the calculated servo angles
            joint_coords = ik_solver.forward_kinematics(angles)
            ik_solver.visualize_arm(joint_coords, x, y, z)
            
            # 2. Format and send command to Arduino
            send_command(ser, angles)


# --- 5. MAIN EXECUTION ---

def run_controller():
    """Initializes serial communication and runs the interactive IK loop."""
    
    # Check for pyserial dependency
    try:
        import serial
    except ImportError:
        print("The 'pyserial' library is required.")
        print("Please install it using: pip install pyserial")
        sys.exit(1)
        
    # Initialize IK Solver and Plot
    ik_solver = ArmIK(L0_Z, L1, L2)
    
    # --- Dynamic Port Selection ---
    selected_port = select_serial_port()
    if selected_port is None:
        print("Serial port selection cancelled. Exiting.")
        sys.exit(0)
    # -----------------------------
    
    # Initialize Serial Port
    try:
        ser = serial.Serial(selected_port, BAUD_RATE, timeout=0.1)
        print(f"--- Connected to Arduino on {selected_port} @ {BAUD_RATE} bps ---")
        time.sleep(2) # Wait for Arduino to reset
        
        # Send initial command to ENABLE and HOME the arm
        ser.write(b'H\n')
        print("Sent 'H' (Home) command to Arduino.")

    except serial.SerialException as e:
        print(f"\n!!! ERROR: Could not open serial port {selected_port}.")
        print("Please check the port name and ensure Arduino is connected and not running the Serial Monitor.")
        print(f"Details: {e}")
        sys.exit(1)

    print(f"\nArm Link Parameters: L1={L1}mm, L2={L2}mm, L0_Z={L0_Z}mm. Max Reach: {MAX_REACH}mm.")
    print("------------------------------------------------------------------")
    
    while True:
        try:
            mode = input("\nSelect Mode: (I)nteractive IK, (J)oint Sweep Test, (L)inear Motion Test, (Q)uit: ")
            
            if mode.lower() == 'q':
                break
            elif mode.lower() == 'i':
                interactive_ik_mode(ik_solver, ser)
            elif mode.lower() == 'j':
                test_joint_sweep(ik_solver, ser)
            elif mode.lower() == 'l':
                test_linear_motion(ik_solver, ser)
            else:
                print("Invalid mode selected. Please choose I, J, L, or Q.")
        
        except KeyboardInterrupt:
            print("\nExiting controller.")
            break
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            time.sleep(1)

    # Clean up
    if 'ser' in locals():
        # Send disable command before closing (optional safety)
        ser.write(b'D\n') 
        ser.close()
    
    if plt:
        plt.close(ik_solver.fig) # Close the visualization window

if __name__ == '__main__':
    run_controller()
