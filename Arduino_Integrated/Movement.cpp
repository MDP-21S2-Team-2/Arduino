#include "Movement.h"
#include "PID.h"
#include "Sensors.h"
#include "Motors.h"
#include "Alignment.h"

bool emergencyBrakes = false;

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
#ifndef EXPLORATION_MODE  // FP
      //checkForCrash();
#endif
    }
  }
  if (!emergencyBrakes)
    md.setBrakes(BRAKE_L, BRAKE_R);
  if (emergencyBrakes) {
    // TODO: perform recovery action?
    //Serial.write("EMERGENCY\n");
    emergencyBrakes = false;
  }
  // reset PID
  resetPIDControllers();
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
  resetPIDControllers();
}

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
    tEncodeVal = 384; //angle * 4.3; // 4.33;  // 4.41: 100 RPM
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
  resetPIDControllers();
}

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
    tEncodeVal = 382; //angle * 4.26; // 4.31; //4.41 for 100 RPM; // 4.42 for paper, 4.41 for arena
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
  resetPIDControllers();
}

void checkAlignmentAfterMove() {
  delay(100); // allow robot to settle

  // ensure robot is centralised within its grids
  checkCentralise_Front();
  checkCentralise_Sides();
  
  // align robot to be straight
  checkForTilted();
}

void checkAlignmentAfterRotate() {
  delay(100); // allow robot to settle
  
  // align robot to be straight
  checkForTilted();
}

// function to calibrate sensors in starting grid
void initialGridCalibration() {

  // 1. rotate left for robot's front to face wall
  rotateLeft(90);
  
  // 3. TBD: wait a short while (for brakes to settle?)
  delay(500);
  
  // 2. align accurately with wall in front
  checkCentralise_Front();
  checkForTilted();

  // 3. TBD: wait a short while (for brakes to settle)
  delay(500);

  // 4. turn left again to face wall behind
  rotateLeft(90);

  // 5. align with wall so robot is correct distance away
  checkCentralise_Front();
//  if (front_D1.getDistance() < 4) { // front-mid sensor
//    alignBack_Front(SharpIR::D1, false, 4.0);
//  }
//  else if (front_D1.getDistance() > 4.5) {  // front-right sensor
//    alignForward_Front(SharpIR::D2, false, 4.0);
//  }

  delay(500);

  // 6. rotate back to face the front
  rotateRight(90);
  delay(500);
  rotateRight(90);
  delay(500);

  
  double dist_S1 = left_S1.getDistance();
  double dist_S2 = left_S2.getDistance();

if ((dist_S1 >= 2.0 && dist_S1 <= 8.0 && dist_S2 >= 3.0 && dist_S2 <= 8.0 && abs(dist_S1 - dist_S2) > 0.1) ||
    (dist_S1 >= 13.0 && dist_S1 <= 18.0 && dist_S2 >= 13.0 && dist_S2 <= 18.0 && abs(dist_S1 - dist_S2) > 0.5) ||
    (dist_S1 >= 23.0 && dist_S1 <= 28.0 && dist_S2 >= 23.0 && dist_S2 <= 28.0 && abs(dist_S1 - dist_S2) > 1.5))
  { // TODO: distance difference threshold
    alignToLeftWall();
  }

  // 5. TBD: align with side sensors?

  // TBD: if sensor readings are too different, do a double-check?

}
