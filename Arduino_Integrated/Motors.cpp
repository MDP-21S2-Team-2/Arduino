#include "Motors.h"
DualVNH5019MotorShield md;

// PID controller for each motor
// parameter list: P, I, D, Imax, Imin
PID leftPIDController(3.649, 1.678, 4.548, 200.0, -200);
PID rightPIDController(3.824, 1.685, 4.49, 200.0, -200.0);

//PID leftPIDController(3.48, 1.625, 4.934, 200.0, -200); //red
//PID rightPIDController(3.87, 1.675, 4.85, 200.0, -200.0);

// 24 Feb target speed: 125 6.26V 1y1w
//PID leftPIDController(3.8, 2.6, 5.2, 130.0, -130); // red
//PID rightPIDController(3.55, 2.6, 5.3, 130.0, -130.0); // right starts up faster

// Distance Function
//double leftWheelDiameter = 6.0;   // in cm
//double rightWheelDiameter = 6.0;  // in cLLm
//Wheel to wheel distance = 18.5cm
//Circumference of whole robot m= 58.11cm
// 90 degree = 14.5275cm

// Left
volatile unsigned long L_prevTime = 0;
volatile unsigned long L_currTime = 0;
volatile unsigned long L_timeWidth = 0;

// Right
volatile unsigned long R_prevTime = 0;
volatile unsigned long R_currTime = 0;
volatile unsigned long R_timeWidth = 0;

volatile unsigned char encL_count = 0;
volatile unsigned char encR_count = 0;
volatile unsigned char encL_curr_count = 0;
volatile unsigned char encR_curr_count = 0;
volatile unsigned char encL_overshootCount = 0;
volatile unsigned char encR_overshootCount = 0;

void resetEnc() {
  encL_count = 0;
  encR_count = 0;
  encR_curr_count = 0;
  encL_curr_count = 0;
  encL_overshootCount = 0;
  encR_overshootCount = 0;
  // reset timeWidths
  L_timeWidth = 0;
  R_timeWidth = 0;
}

void resetPIDControllers() {
  leftPIDController.resetPID();
  rightPIDController.resetPID();
}

double calculateRpm(int pulseWidth) {
  if (pulseWidth == 0)
    return 0;
  else {
    //return 60000000.0 / (pulseWidth  * 56.225);  // time for 1 revolution in seconds
    return 533570.48 / pulseWidth;
  }
  //to get RPM, 60 / <above value>
}

void motorR_ISR() {
  encR_count++;
  if (encR_count == 0)
    encR_overshootCount++;
  encR_curr_count++;
  //For every 10 encoder count
  if (encR_curr_count == 1)
  {
    R_prevTime = micros();
  }
  else if (encR_curr_count == 6)
  {
    R_currTime = micros();
    R_timeWidth = alpha * (R_currTime - R_prevTime) + alphaInv * R_timeWidth;
    encR_curr_count = 0;
    R_prevTime = R_currTime;
  }
}

void motorL_ISR() {
  encL_count++;
  if (encL_count == 0)
    encL_overshootCount++;
  encL_curr_count++;
  if (encL_curr_count == 1)
  {
    L_prevTime = micros();
  }
  else if (encL_curr_count == 6)
  {
    L_currTime = micros();
    L_timeWidth = alpha * (L_currTime - L_prevTime) + alphaInv * L_timeWidth;
    encL_curr_count = 0;
    L_prevTime = L_currTime;
  }
}
