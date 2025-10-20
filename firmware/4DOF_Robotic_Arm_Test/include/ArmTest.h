#ifndef ARMTEST_H
#define ARMTEST_H

#include "ArmController.h"

// Number of joints (Base, Shoulder, Elbow, Wrist)
const int NUM_JOINTS = 4;

// Function prototypes for the test utility
void runInteractiveTest(ArmController& armController);
void printTestInstructions();

#endif // ARMTEST_H
