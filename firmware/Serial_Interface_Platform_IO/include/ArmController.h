#ifndef ARMCONTROLLER_H
#define ARMCONTROLLER_H

#include <Arduino.h>

// Define the pin mapping based on the provided Timer configuration:
// D9 (Base) -> Timer1 OC1A
// D10 (Shoulder) -> Timer1 OC1B
// D11 (Elbow) -> Timer2 OC2A
// D3 (Wrist) -> Timer2 OC2B
// D2 (Enable) -> Custom pin for MOSFET/H-bridge control

class ArmController {
public:
    // Constructor initializes pin numbers
    ArmController(int baseP, int shoulderP, int elbowP, int wristP, int enableP);

    // Main setup function (configures timers and sets home position)
    void initialize();
    
    // Control functions
    void enableServos();
    void disableServos();
    bool isEnabled() const { return isPowered; } 
    void goToPosition(int baseAngle, int shoulderAngle, int elbowAngle, int wristAngle);
    void homeArm(int stepDelay = 5); // stepDelay is used for the settling time

    // Readback function
    int getCurrentAngle(int index); // 0=Base, 1=Shoulder, 2=Elbow, 3=Wrist
    
private:
    const int basePin;
    const int shoulderPin;
    const int elbowPin;
    const int wristPin;
    const int enablePin;
    
    bool isPowered;

    // Timer configuration and servo control
    void initServosTimer();
    int angleToTimerValue(int angle, int timer_id);
};

// Global Constants for Home Position
namespace ArmConstants {
    extern const int HOME_BASE; 
    extern const int HOME_SHOULDER; 
    extern const int HOME_ELBOW; 
    extern const int HOME_WRIST; 
}

#endif // ARMCONTROLLER_H