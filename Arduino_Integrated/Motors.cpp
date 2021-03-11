#include "Motors.h"
DualVNH5019MotorShield md;

// PID controller for each motor
// parameter list: P, I, D, Imax, Imin

// 125 RPM; battery 1y1w 6.33V, now 6.37V
//PID leftPIDController(3.74, 1.65, 4.69, 200.0, -200);
//PID rightPIDController(3.97, 1.69, 4.76, 200.0, -200.0); // P: 3.8

//rpm = 110, 1y1w, 6.37-39V
PID leftPIDController(3.3, 1.447, 4.532, 200.0, -200);//red
PID rightPIDController(3.1, 1.395, 4.5, 200.0, -200.0); // P: 3.04 for 6.42V

//rpm = 120, 1y1w, 6.38V
//PID leftPIDController(3.38, 1.58, 4.85, 200.0, -200);//red
//PID rightPIDController(3.4, 1.62, 4.73, 200.0, -200.0);

//rpm = 120, 1y1w, 6.36V - cardboard added
//PID leftPIDController(3.62, 1.58, 4.86, 200.0, -200);//red
//PID rightPIDController(3.45, 1.62, 4.75, 200.0, -200.0);

// with exponential smoothing filter
//PID leftPIDController(1.0, 1.439, 0.2, 200.0, -200);//red
//PID rightPIDController(0.52, 1.46, 0.4, 200.0, -200.0);

//PID leftPIDController(1.2, 1.4, 2.0, 200.0, -200, 0, 400);//red
//PID rightPIDController(1.0, 1.386, 2.5, 200.0, -200.0, 0, 400);

// Distance Function
//double leftWheelDiameter = 6.0;   // in cm
//double rightWheelDiameter = 6.0;  // in cm
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
//    R_timeWidth = R_currTime - R_prevTime;
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
//    L_timeWidth = L_currTime - L_prevTime;
    encL_curr_count = 0;
    L_prevTime = L_currTime;
  }
}
