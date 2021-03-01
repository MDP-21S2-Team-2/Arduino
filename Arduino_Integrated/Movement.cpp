#include "Movement.h"
#include "PID.h"
#include "AlignAndCheck.h"
#include "Sensors.h"
#include <DualVNH5019MotorShield.h>
#define targetRpm 125.0
#define MOVE_OFFTICKS 24
#define BRAKE_L 400
#define BRAKE_R 400

bool emergencyBrakes = false;
DualVNH5019MotorShield md;
// Left
volatile unsigned long L_prevTime = 0;
volatile unsigned long L_currTime = 0;
volatile unsigned long L_timeWidth = 0;

// Right
volatile unsigned long R_prevTime = 0;
volatile unsigned long R_currTime = 0;
volatile unsigned long R_timeWidth = 0;

volatile int encL_count = 0;
volatile int encR_count = 0;
volatile int encL_curr_count =0;
volatile int encR_curr_count =0;

PID leftPIDController(3.48, 1.625, 4.934, 200.0, -200); //red
PID rightPIDController(3.87, 1.675, 4.85, 200.0, -200.0);

void resetEnc() {
  encL_count = 0;
  encR_count = 0;
  encR_curr_count = 0;
  encL_curr_count = 0;
  // reset timeWidths
  L_timeWidth = 0;
  R_timeWidth = 0;
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

void moveForward(int moveUnits)
{
  // reset encoder ticks
  resetEnc();
  
  //double tEncodeVal = tDistance / oneRevDis * 562.25 - MOVE_OFFTICKS; // Calculate target number of ticks to travel the distance & reduce tEncodeVal by no. ticks needed for braking
  int tEncodeVal = tEncodeVal_lut[moveUnits];

  // reset prevTime to get more accurate timeWidth
//  L_prevTime = micros();
//  R_prevTime = micros();

  // check if either motor reached the target number of ticks
  while (!emergencyBrakes && ((encL_count <= tEncodeVal) || (encR_count <= tEncodeVal)))
  //while (0.5*(encL_count + encR_count) <= tEncodeVal)
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
      //md.setSpeeds(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm), rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));

      // read IR sensors here to check for emergency brakes
      checkForCrashCalibration();
      // TODO: send IR sensor data to algo?
    }
  }
  if (!emergencyBrakes)
    md.setBrakes(BRAKE_L, BRAKE_R);
//  if (emergencyBrakes) {
//    // TODO: perform recovery action
//    Serial.write("EMERGENCY\n");
//    emergencyBrakes = false;
//  }
  // reset PID
  leftPIDController.resetPID();
  rightPIDController.resetPID();
  // TODO: check if robot is aligned
  checkForAlignmentCalibration();
}
void moveBackward(int moveUnits)
{
  // reset encoder ticks
  resetEnc();
  //double tEncodeVal = tDistance / oneRevDis * 562.25 - MOVE_OFFTICKS; // Calculate target number of ticks to travel the distance & reduce tEncodeVal by no. ticks needed for braking
  int tEncodeVal = tEncodeVal_lut[moveUnits];
  // reset prevTime to get more accurate timeWidth
//  L_prevTime = micros();
//  R_prevTime = micros();
  // check if either motor reached the target number of ticks
  while ((encL_count <= tEncodeVal) || (encR_count <= tEncodeVal))
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(-rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
  
  // reset PID
  leftPIDController.resetPID();
  rightPIDController.resetPID();
}

//void rotateLeft2(double angle) {
//  // reset encoder ticks
//  resetEnc();
//  // Calculate target number of ticks to travel the distance,
//  // reduce tEncodeVal by no. ticks needed for braking
//  // was previously 4.45
//  double tEncodeVal = 0;
//  if (angle <= 45) {
//    tEncodeVal = angle * 4.21;  // 4.27 works
//  }
//  else if (angle <= 91) {
//  // 4.42 FKING SOLID 11 FEB 6.34V TUNING
//    tEncodeVal += angle * 4.42;
//  }
//  else if (angle <= 181) {
//    tEncodeVal += angle * 4.51;
//  }
//
//  // reset prevTime to get more accurate timeWidth
//  L_prevTime = micros();
//  R_prevTime = micros();
//
//  while ((encL_count <= tEncodeVal) || (encR_count <= tEncodeVal))
//  {
//    if (PID::checkPIDCompute()) {
//      md.setM1Speed(leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
//      md.setM2Speed(rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
//      //md.setSpeeds(leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm), rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
//    }
//  }
//  md.setBrakes(BRAKE_L, BRAKE_R);
//  // reset PID
//  leftPIDController.resetPID();
//  rightPIDController.resetPID();
//}

void rotateLeft(int angle)
{
  // reset encoder ticks
  resetEnc();
  int tEncodeVal = 0;
  //4.8147 exact multiplier
  // Calculate target number of ticks to travel the distance,
  // reduce tEncodeVal by no. ticks needed for braking
  // Every degree takes (1/360 *58.11 /18.84956 * 562.25) = 4.8147
  // check if either motor reached the target number of ticksif (angle <=90)
  if (angle == 90)
    tEncodeVal = 387; //angle * 4.3; // 4.33;  // 4.41: 100 RPM
  else if (angle == 180)
    tEncodeVal = 810; //angle * 4.5;  // 4.65

  // reset prevTime to get more accurate timeWidth
//  L_prevTime = micros();
//  R_prevTime = micros();

  while ((encL_count + encR_count) / 2 <= tEncodeVal)
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
      //md.setSpeeds(leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm), rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
  
  // reset PID
  leftPIDController.resetPID();
  rightPIDController.resetPID();
}

//void rotateRight2(double angle) // doesn't really work but I'll leave it here
//{
//  // reset encoder ticks
//  resetEnc();
//  //4.8147 exact multiplier
//  // Calculate target number of ticks to travel the distance,
//  // reduce tEncodeVal by no. ticks needed for braking
//  double tEncodeVal = angle * 4.42; //4.46; for brakes: //- 35;  // 38
//  // reset prevTime to get more accurate timeWidth
//  L_prevTime = micros();
//  R_prevTime = micros();
//  while ((encL_count <= tEncodeVal) || (encR_count <= tEncodeVal))
//  {
//    if (PID::checkPIDCompute()) {
//      md.setM1Speed(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
//      md.setM2Speed(-rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
//      //md.setSpeeds(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm), -rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
//    }
//  }
//  md.setBrakes(BRAKE_L, BRAKE_R);
//  
//  // reset PID
//  leftPIDController.resetPID();
//  rightPIDController.resetPID();
//}

void rotateRight(int angle)
{
  // reset encoder ticks
  resetEnc();
  int tEncodeVal = 0;
  //4.8147 exact multiplier
  // Calculate target number of ticks to travel the distance,
  // reduce tEncodeVal by no. ticks needed for braking
  // Every degree takes (1/360 *58.11 /18.84956 * 562.25) = 4.8147
  // check if either motor reached the target number of ticksif (angle <=90)
  if (angle == 90)
    tEncodeVal = 384; //angle * 4.26; // 4.31; //4.41 for 100 RPM; // 4.42 for paper, 4.41 for arena
  else if (angle == 180)
    tEncodeVal = 806; //angle * 4.48;

  // reset prevTime to get more accurate timeWidth
//  L_prevTime = micros();
//  R_prevTime = micros();

  while ((encL_count + encR_count) / 2 <= tEncodeVal)
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(-rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
      //md.setSpeeds(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm), -rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
  
  // reset PID
  leftPIDController.resetPID();
  rightPIDController.resetPID();
}
