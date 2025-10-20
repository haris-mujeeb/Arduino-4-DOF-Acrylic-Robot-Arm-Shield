#include "ArmTest.h"
#include "ArmController.h"
#include <Arduino.h>

// Global state variables for the test utility
static int currentAngles[NUM_JOINTS];
static int selectedJoint = 0; // 0=Base, 1=Shoulder, 2=Elbow, 3=Wrist
const int STEP_SIZE = 5; // Degrees to move per 'I' or 'K' command

/**
 * @brief Updates the robot's position based on the global currentAngles array.
 */
void updateRobotPosition(ArmController& armController) {
    // Call the high-performance goToPosition function
    armController.goToPosition(
        currentAngles[0],
        currentAngles[1], 
        currentAngles[2], 
        currentAngles[3]
    );
}

/**
 * @brief Displays the current status of all joints, enable pin, and the selected joint.
 */
void printStatus(ArmController& armController) {
    // Read the *true* current angle from the hardware registers for display
    // The currentAngles array is synced here with the hardware state
    for (int i = 0; i < NUM_JOINTS; i++) {
        currentAngles[i] = armController.getCurrentAngle(i);
    }
    
    Serial.println("---------------------------------------------------------"); 
    
    // 1. Enable Pin Status (uses the new isEnabled() method)
    Serial.print("D2 STATUS | Enable Pin: ");
    Serial.println(armController.isEnabled() ? "HIGH (POWER ON) ✅" : "LOW (POWER OFF) ❌");

    // 2. Selected Motor
    Serial.print("SELECTED  | Joint: ");
    switch (selectedJoint) {
        case 0: Serial.print("1: BASE (D9)    "); break;
        case 1: Serial.print("2: SHOULDER (D10)"); break;
        case 2: Serial.print("3: ELBOW (D11)  "); break;
        case 3: Serial.print("4: WRIST (D3)   "); break;
    }
    
    // 3. Motor Angles
    Serial.print(" | Angle: ");
    Serial.println(currentAngles[selectedJoint]);
    
    // 4. Full Angle Set
    Serial.print("ANGLES    | [B, S, E, W]: [");
    for (int i = 0; i < NUM_JOINTS; i++) {
        Serial.print(currentAngles[i]);
        if (i < NUM_JOINTS - 1) Serial.print(", ");
    }
    Serial.println("]");
    Serial.println("---------------------------------------------------------");
}


/**
 * @brief Prints instructions for the serial interactive test.
 */
void printTestInstructions() {
    Serial.println("--- COMMANDS ---");
    Serial.println("1, 2, 3, 4: Select Joint (Base, Shoulder, Elbow, Wrist)");
    Serial.println("I / +: Increase selected joint angle by 5 degrees");
    Serial.println("K / -: Decrease selected joint angle by 5 degrees");
    Serial.println("H: Home all joints (ENABLE D2 & move to preset Home)");
    Serial.println("?: Show this help menu");
    Serial.println("----------------");
}

/**
 * @brief Runs the interactive serial monitor test for the robotic arm.
 */
void runInteractiveTest(ArmController& armController) {
    // Initialize currentAngles from the controller's initial register state
    static bool initialized = false;
    if (!initialized) {
        // Sync local angle array with the initial hardware state
        for (int i = 0; i < NUM_JOINTS; i++) {
            currentAngles[i] = armController.getCurrentAngle(i);
        }
        initialized = true;
        printTestInstructions();
        printStatus(armController);
    }
    
    if (Serial.available() > 0) {
        char command = Serial.read();
        bool status_changed = true; 

        // Convert to uppercase for case-insensitive commands
        if (command >= 'a' && command <= 'z') {
            command = command - ('a' - 'A');
        }

        // If the arm is disabled, only allow 'H' or '?'
        if (!armController.isEnabled() && command != 'H' && command != '?') {
            Serial.println("Error: Servos are DISABLED. Press 'H' to enable and home.");
            return;
        }

        switch (command) {
            // Joint Selection
            case '1':
            case '2':
            case '3':
            case '4':
                selectedJoint = command - '1';
                break;

            // Increase Angle
            case 'I': 
            case '+':
                currentAngles[selectedJoint] = currentAngles[selectedJoint] + STEP_SIZE;
                if (currentAngles[selectedJoint] > 180) currentAngles[selectedJoint] = 180;
                updateRobotPosition(armController);
                break;

            // Decrease Angle
            case 'K':
            case '-':
                currentAngles[selectedJoint] = currentAngles[selectedJoint] - STEP_SIZE;
                if (currentAngles[selectedJoint] < 0) currentAngles[selectedJoint] = 0;
                updateRobotPosition(armController);
                break;

            // Homing Sequence
            case 'H':
                armController.homeArm(50); 
                // Force sync the local angles array to the home angles
                for (int i = 0; i < NUM_JOINTS; i++) {
                    currentAngles[i] = armController.getCurrentAngle(i);
                }
                break;
            
            // Help/Status Commands
            case '?':
                printTestInstructions();
                // Fall-through to show status after printing instructions
            case '\n': // Ignore newline characters
            case '\r': // Ignore carriage return characters
                status_changed = false; 
                break;
            
            default:
                status_changed = false; 
                break;
        }
        
        // Print the full status after every command that modifies state or requires feedback
        if (status_changed) {
            printStatus(armController);
        }
    }
}
