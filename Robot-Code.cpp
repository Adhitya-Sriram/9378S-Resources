#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition) \
do { \
wait(5, msec); \
} while (!(condition))

#define repeat(iterations) \
for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
digital_out clamp = digital_out(Brain.ThreeWirePort.A);
controller Controller1 = controller(primary);
motor l1 = motor(PORT8, ratio6_1, false);
motor l2 = motor(PORT2, ratio6_1, true);
motor l3 = motor(PORT4, ratio6_1, true);
motor r1 = motor(PORT7, ratio6_1, true);
motor r2 = motor(PORT5, ratio6_1, false);
motor r3 = motor(PORT6, ratio6_1, false);
motor conv = motor(PORT19, ratio18_1, false);
motor ladybrown = motor(PORT10, ratio36_1, false);
motor_group leftDrive = motor_group(l1,l2,l3);
motor_group rightDrive = motor_group(r1,r2,r3);
drivetrain Drivetrain = drivetrain(leftDrive,rightDrive);
encoder ladybrownSensor = encoder(Brain.ThreeWirePort.G);



// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
printf("VEXPlaySound:%s\n", soundName);
wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

/*----------------------------------------------------------------------------*/
/* */
/* Module: main.cpp */
/* Author: {RK} */
/* Created: {2025} */
/* Description: Sigma code */
/* */
/*----------------------------------------------------------------------------*/

// Include the V5 Library
#include "vex.h"
using namespace vex;
// PID Constants
double kP = 0.2; // Proportional constant
double kI = 0.01; // Integral constant
double kD = 0.1; // Derivative constant
double kp = 0.2;
int cState = 0;
int target = 0;
int states[] = {0,60,475};
int c = 0;
double leftError, rightError; // Error terms for PID
double leftPrevError = 0, rightPrevError = 0; // Previous error terms
double leftIntegral = 0, rightIntegral = 0; // Integral terms
double leftDerivative, rightDerivative; // Derivative terms
double leftSpeed = 0, rightSpeed = 0; // Output speeds

// Lady Brorn PID Loop
void next(void){
cState++;
if(cState == 3){
cState = 0;
}
target = states[cState];
}
void liftC(void){
double e = target - ladybrownSensor.position(degrees);
double v = kp * e;
ladybrown.setVelocity(v, percent);
ladybrown.spin(forward, e*kp, volt);
}
void runRotationC(void){
while(true){
liftC();
next();
wait(0.01, seconds);
}
}


// Function to calculate PID for drivetrain
void autonomous() {
double targetLeft = 1.0; // Vertical axis for left drive
double targetRight = 1.0; // Vertical axis for right drive

// Calculate errors
leftError = targetLeft - leftDrive.velocity(percent);
rightError = targetRight - rightDrive.velocity(percent);

// Calculate integral (with anti-windup)
leftIntegral += leftError;
rightIntegral += rightError;
leftIntegral = fabs(leftError) < 5 ? leftIntegral : 0; // Reset if error too large
rightIntegral = fabs(rightError) < 5 ? rightIntegral : 0;

// Calculate derivative
leftDerivative = leftError - leftPrevError;
rightDerivative = rightError - rightPrevError;

// PID calculations
leftSpeed = (kP * leftError) + (kI * leftIntegral) + (kD * leftDerivative);
rightSpeed = (kP * rightError) + (kI * rightIntegral) + (kD * rightDerivative);

// Apply speeds
leftDrive.spin(forward, leftSpeed, percent);
rightDrive.spin(forward, rightSpeed, percent);

// Update previous errors
leftPrevError = leftError;
rightPrevError = rightError;
}

void userControl(void){
thread a = thread(liftC);
while (true) {
int drivetrainLeftSideSpeed = Controller1.Axis3.position() + (Controller1.Axis1.position()/2);
int drivetrainRightSideSpeed = Controller1.Axis3.position() - (Controller1.Axis1.position()/2);
if(true){
rightDrive.setVelocity(drivetrainRightSideSpeed, percent);
rightDrive.spin(forward);
}
if(true){
leftDrive.setVelocity(drivetrainLeftSideSpeed, percent);
leftDrive.spin(forward);
}
// Control intake and conveyor
if (Controller1.ButtonL1.pressing()) {
conv.spin(forward, 200, percent);
} else if (Controller1.ButtonL2.pressing()) {
conv.spin(reverse, 200, percent);
} else {
conv.stop();
}

// Control Lady Brown mechanism
if (Controller1.ButtonR2.pressing()){
if(c>1){
c=0;
}
if(c==0){
clamp.set(true);
c++;
wait(0.01, seconds);
}else{
clamp.set(false);
c++;
wait(0.01, seconds);
}
wait(1, seconds);
}

Controller1.ButtonR1.pressed(runRotationC);

}


}

// Main program
int main() {

competition Competition;
conv.setVelocity(100,percent);
ladybrown.setVelocity(50, percent);
Competition.autonomous(autonomous);
Competition.drivercontrol(userControl);
return 0;
}
