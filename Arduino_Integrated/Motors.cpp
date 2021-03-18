#include "Motors.h"
DualVNH5019MotorShield md;

// PID controller for each motor
// parameter list: P, I, D, Imax, Imin

#if targetRpm == TARGETRPM_110
  //rpm = 110, 1y1w, 6.35-36V
  PID leftPIDController(3.3, 1.447, 4.532, 200.0, -200);//red
  PID rightPIDController(3.1, 1.41, 4.5, 200.0, -200.0); // P: 3.04 for 6.42V
#elif targetRpm == TARGETRPM_120
  //rpm = 120, 1y1w, 6.36V - cardboard added
  // 6.38V - 2y
  // 6.43V - 1y1w, 6.4V after turned on
  PID leftPIDController(3.72, 1.58, 4.86, 200.0, -200);//red
  PID rightPIDController(3.5, 1.545, 4.8, 200.0, -200.0);
#endif

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


// compute no. units moved forward
int computeUnitsMoved() {

  // get average no. ticks moved of both encoders
  int ticksL = encL_overshootCount * 256 + encL_count;
  int ticksR = encR_overshootCount * 256 + encR_count;
  int ticksAverage = 0.5 * (ticksL + ticksR);
  // compute no. units moved
  double distance = (ticksAverage + MOVE_OFFTICKS) / 562.25 * ONEREVDIST;
  int units = (distance + 5.0) * 0.1;
  return units;
}

void sendUnitsMoved(int units) {
  
  byte* ptr_units = (byte*) &units;

  Serial.write("G,");
  Serial.write(ptr_units, 2);
  Serial.write(',');
}
