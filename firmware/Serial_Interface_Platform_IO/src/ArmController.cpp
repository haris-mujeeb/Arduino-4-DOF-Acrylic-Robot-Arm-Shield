#include "ArmController.h"
#include <Arduino.h> 
#include <avr/io.h> // Include for direct register access

// Define the new HOME (Parked) Position angles constants. 
namespace ArmConstants {
    // New Home positions (same as requested)
    const int HOME_BASE = 25;   
    const int HOME_SHOULDER = 0; 
    const int HOME_ELBOW = 60;  
    const int HOME_WRIST = 180; 
}

// ------------------------------------------------------------------
// UTILITY FUNCTIONS (Timer Mapping)
// ------------------------------------------------------------------

/**
 * @brief Maps an angle (0-180) to the required Timer Compare Value.
 * This conversion is critical for translating desired angles to OCR register values.
 */
int ArmController::angleToTimerValue(int angle, int timer_id) {
    // Clamp angle to valid range (0-180)
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Timer 1 (16-bit) mapping for D9 (OC1A) and D10 (OC1B)
    // Range: 50Hz, 125 (500us) to 625 (2500us). This uses 16-bit resolution.
    if (timer_id == 1) {
        return map(angle, 0, 180, 125, 625);
    } 
    
    // Timer 2 (8-bit) mapping for D11 (OC2A) and D3 (OC2B)
    // Range: ~61Hz, 8 (approx 500us) to 39 (approx 2500us). This uses 8-bit resolution.
    if (timer_id == 2) {
        return map(angle, 0, 180, 8, 39); 
    }
    return 0;
}

// ------------------------------------------------------------------
// HARDWARE INITIALIZATION (REGISTER CONTROL)
// ------------------------------------------------------------------

/**
 * @brief Configures Timer1 (D9, D10) and Timer2 (D3, D11) for stable PWM generation.
 */
void ArmController::initServosTimer() {
    // --- TIMER 1 (16-bit, Pins D9, D10) ---
    // Mode 14: Fast PWM, TOP = ICR1, Prescaler N=64 -> Freq = 50Hz (20ms period)

    TCCR1A = 0; 
    TCCR1B = 0; 
    ICR1 = 4999; // Sets TOP value: (16M / (64 * 50Hz)) - 1 = 4999
    
    TCCR1B |= (1 << WGM13) | (1 << WGM12); // Mode 14
    TCCR1A |= (1 << WGM11);
    
    // Compare Output Mode: Clear OC1A/B on Compare Match, Set at TOP
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1); 
    
    TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64, start timer
    
    // --- TIMER 2 (8-bit, Pins D11, D3) ---
    // Mode 3: Fast PWM, TOP = 0xFF, Prescaler N=1024 -> Freq = ~61Hz

    TCCR2A = 0; 
    TCCR2B = 0;
    TCCR2A |= (1 << WGM21) | (1 << WGM20); // Mode 3
    
    // Compare Output Mode: Clear OC2A/B on Compare Match, Set at TOP
    TCCR2A |= (1 << COM2A1) | (1 << COM2B1); 
    
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024, start timer
    
    // Set all PWM pins as output using Arduino functions for clarity
    pinMode(basePin, OUTPUT);
    pinMode(shoulderPin, OUTPUT);
    pinMode(elbowPin, OUTPUT);
    pinMode(wristPin, OUTPUT);

    Serial.println("ArmController: Hardware Timers Configured.");
}

// ------------------------------------------------------------------
// CONSTRUCTOR AND INITIALIZATION
// ------------------------------------------------------------------

ArmController::ArmController(int baseP, int shoulderP, int elbowP, int wristP, int enableP)
    : basePin(baseP), shoulderPin(shoulderP), elbowPin(elbowP), wristPin(wristP), enablePin(enableP),
      isPowered(false) {}

/**
 * @brief Sets up pin modes, initializes timers, and ensures D2 is LOW (DISABLED).
 */
void ArmController::initialize() {
    Serial.println("ArmController: Initializing Servo Enable Pin (D2).");
    
    // 1. Set D2 as OUTPUT and ensure it starts LOW (DISABLED)
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW); 

    // 2. Configure the hardware timers for PWM signal generation
    initServosTimer();
    
    // 3. Write the initial HOME position to the registers (power still LOW)
    goToPosition(ArmConstants::HOME_BASE, ArmConstants::HOME_SHOULDER, ArmConstants::HOME_ELBOW, ArmConstants::HOME_WRIST);
    
    Serial.println("ArmController: PWM signals set to Home position (D2 LOW/DISABLED).");
}

// ------------------------------------------------------------------
// PUBLIC API
// ------------------------------------------------------------------

void ArmController::enableServos() {
    if (!isPowered) {
        digitalWrite(enablePin, HIGH); // Apply power/torque
        isPowered = true;
        Serial.println("ArmController: Servos Power ENABLED (D2 HIGH).");
    }
}

void ArmController::disableServos() {
    if (isPowered) {
        digitalWrite(enablePin, LOW); // Remove power/torque
        isPowered = false;
        Serial.println("ArmController: Servos Power DISABLED (D2 LOW).");
    }
}

/**
 * @brief Writes directly to the hardware OCR registers to set the servo positions.
 */
void ArmController::goToPosition(int baseAngle, int shoulderAngle, int elbowAngle, int wristAngle) {
    // Write angles to the appropriate Output Compare Registers (OCR)
    
    // Base (D9) is Timer1 OC1A
    OCR1A = angleToTimerValue(baseAngle, 1);
    
    // Shoulder (D10) is Timer1 OC1B
    OCR1B = angleToTimerValue(shoulderAngle, 1);

    // Elbow (D11) is Timer2 OC2A
    OCR2A = angleToTimerValue(elbowAngle, 2);
    
    // Wrist (D3) is Timer2 OC2B
    OCR2B = angleToTimerValue(wristAngle, 2);
    
    // Note: No Serial logging here for maximum speed.
}

/**
 * @brief Sets the PWM signal to the HOME position, then enables torque (D2 HIGH).
 */
void ArmController::homeArm(int stepDelay) {
    // If the arm is already powered, don't re-run the startup sequence.
    if (isPowered) return; 

    Serial.println("ArmController: Applying power to move to pre-set Home Position...");
    
    // The PWM signal is already set in initialize(). We just need to apply power.
    delay(5); // Small delay to ensure the current PWM cycle is running before power is applied.
    
    // Enable physical power/torque (D2 HIGH) to execute the move
    enableServos(); 

    // Give the arm time to settle into the position after power is applied
    delay(stepDelay * 20); 

    Serial.println("ArmController: Homing Sequence Complete (D2 is HIGH).");
}

/**
 * @brief Reads the current angle from the hardware OCR registers and maps it back.
 */
int ArmController::getCurrentAngle(int index) {
    // Read the raw OCR value and map it back to 0-180
    if (index == 0) return map(OCR1A, 125, 625, 0, 180); // Base (D9)
    if (index == 1) return map(OCR1B, 125, 625, 0, 180); // Shoulder (D10)
    if (index == 2) return map(OCR2A, 8, 39, 0, 180);    // Elbow (D11)
    if (index == 3) return map(OCR2B, 8, 39, 0, 180);    // Wrist (D3)
    return 90; 
}
