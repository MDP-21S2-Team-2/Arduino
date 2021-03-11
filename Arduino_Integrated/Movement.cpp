#include "Movement.h"
#include "PID.h"
#include "Sensors.h"
#include "Motors.h"
#include "Alignment.h"

bool emergencyBrakes = false;

void moveForward(int moveUnits, bool emergencyEnabled)
{
  // reset encoder ticks
  resetEnc();
  
  //double tEncodeVal = tDistance / oneRevDis * 562.25 - MOVE_OFFTICKS; // Calculate target number of ticks to travel the distance & reduce tEncodeVal by no. ticks needed for braking
  //int tEncodeVal = tEncodeVal_lut[moveUnits];
  int numOvershoot = numOvershoot_lut[moveUnits];
  int remainderCount = remainderCount_lut[moveUnits];

  // check if either motor reached the target number of ticks
  //while (!emergencyBrakes && ((encL_count <= tEncodeVal) || (encR_count <= tEncodeVal)))
  while (!emergencyBrakes && 
    (((encL_overshootCount < numOvershoot) || (encL_count <= remainderCount)) && ((encR_overshootCount < numOvershoot) || (encR_count <= remainderCount)))
  )
  //while (0.5*(encL_count + encR_count) <= tEncodeVal)
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
      //md.setSpeeds(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm), rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));

      // read IR sensors here to check for emergency brakes
    if (emergencyEnabled)
        //checkForCrash();
        checkForCrashAndRecover(numOvershoot, remainderCount);
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
  //int tEncodeVal = tEncodeVal_lut[moveUnits];
  int numOvershoot = numOvershoot_lut[moveUnits];
  int remainderCount = remainderCount_lut[moveUnits];
  
  // check if either motor reached the target number of ticks
  //while ((encL_count <= tEncodeVal) || (encR_count <= tEncodeVal))
  while (!emergencyBrakes && 
    (((encL_overshootCount < numOvershoot) || (encL_count <= remainderCount)) || ((encR_overshootCount < numOvershoot) || (encR_count <= remainderCount)))
  )
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
  //int tEncodeVal = 0;
  int numOvershoot = 0;
  int remainderCount = 0;
  //4.8147 exact multiplier
  // Calculate target number of ticks to travel the distance,
  // reduce tEncodeVal by no. ticks needed for braking
  // Every degree takes (1/360 *58.11 /18.84956 * 562.25) = 4.8147
  // check if either motor reached the target number of ticksif (angle <=90)
  if (angle == 90) {
    //tEncodeVal = 387; //angle * 4.3; // 4.33;  // 4.41: 100 RPM
    numOvershoot = 1;
    remainderCount = 139;//target 125: 127;// target 110: 139; //123;
  }
  else if (angle == 180) {
    //tEncodeVal = 810; //angle * 4.5;  // 4.65
    numOvershoot = 3;
    remainderCount = 42;
  }

//#ifndef EXPLORATION_MODE
//  remainderCount += 5;
//#endif

  // reset encoder ticks
  resetEnc();
  
  //while ((encL_count <= tEncodeVal) && (encR_count <= tEncodeVal))//((encL_count + encR_count) / 2 <= tEncodeVal)
  while (((encL_overshootCount < numOvershoot) || (encL_count <= remainderCount)) || ((encR_overshootCount < numOvershoot) || (encR_count <= remainderCount)))
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
  //int tEncodeVal = 0;
  int numOvershoot = 0;
  int remainderCount = 0;
  //4.8147 exact multiplier
  // Calculate target number of ticks to travel the distance,
  // reduce tEncodeVal by no. ticks needed for braking
  // Every degree takes (1/360 *58.11 /18.84956 * 562.25) = 4.8147
  // check if either motor reached the target number of ticksif (angle <=90)
  if (angle == 90) {
    //tEncodeVal = 387; //angle * 4.26; // 4.31; //4.41 for 100 RPM; // 4.42 for paper, 4.41 for arena
    numOvershoot = 1;
    remainderCount = 142;//132;
  }
  else if (angle == 180) {
    //tEncodeVal = 806; //angle * 4.48;
    numOvershoot = 3;
    remainderCount = 38;
  }

//#ifndef EXPLORATION_MODE
//  remainderCount += 5;
//#endif

  // reset prevTime to get more accurate timeWidth
//  L_prevTime = micros();
//  R_prevTime = micros();
  
  // reset encoder ticks
  resetEnc();
  
  //while ((encL_count <= tEncodeVal) && (encR_count <= tEncodeVal))//((encL_count + encR_count) / 2 <= tEncodeVal)
  while (((encL_overshootCount < numOvershoot) || (encL_count <= remainderCount)) && ((encR_overshootCount < numOvershoot) || (encR_count <= remainderCount)))
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

void checkAlignmentAfterCommand_FP() {
  // align robot to be straight
  checkForTilted();
}

void checkAlignmentAfterMove() {
  // ensure robot is centralised within its grids
  checkCentralise_Front();
  checkCentralise_Sides();
  
  // align robot to be straight
  checkForTilted();
}

void checkAlignmentAfterRotate() {
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
  delay(500);

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

if ((dist_S1 >= 2.0 && dist_S1 <= 8.0 && dist_S2 >= 3.0 && dist_S2 <= 8.0 && ((dist_S2 - dist_S1) > 0.1 || (dist_S2 - dist_S1) < 0.075)) ||
    (dist_S1 >= 13.0 && dist_S1 <= 18.0 && dist_S2 >= 13.0 && dist_S2 <= 18.0 && abs(dist_S1 - dist_S2) > 0.3) ||
    (dist_S1 >= 23.0 && dist_S1 <= 28.0 && dist_S2 >= 23.0 && dist_S2 <= 28.0 && abs(dist_S1 - dist_S2) > 0.8))
  { // TODO: distance difference threshold
    alignToLeftWall();
  }

  // 5. TBD: align with side sensors?

  // TBD: if sensor readings are too different, do a double-check?

}



// CUSTOM MOVEMENT
void moveForward_custom(double distance, bool emergencyEnabled)
{
  // reset encoder ticks
  resetEnc();
  
  int tEncodeVal = distance / oneRevDis * 562.25 - 24;
  int numOvershoot = tEncodeVal / 256;
  int remainderCount = tEncodeVal % 256;

  // check if either motor reached the target number of ticks
  while (!emergencyBrakes &&
  (((encL_overshootCount < numOvershoot) || (encL_count <= remainderCount)) && ((encR_overshootCount < numOvershoot) || (encR_count <= remainderCount))))
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
      //md.setSpeeds(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm), rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));

    // read IR sensors here to check for emergency brakes
    if (emergencyEnabled)
        //checkForCrash();
        checkForCrashAndRecover(numOvershoot, remainderCount);
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

void rotateRight_custom(int angle, int tickOffset)
{
//  int tEncodeVal = angle * 4.3; //angle * 4.26; // 4.31; //4.41 for 100 RPM; // 4.42 for paper, 4.41 for arena
//  int numOvershoot = tEncodeVal / 256;
//  int remainderCount = tEncodeVal % 256;
  if (angle == 90) {
    numOvershoot = 1;
    remainderCount = 142;//132;
  }
  remainderCount += tickOffset;
  
  // reset encoder ticks
  resetEnc();
  
  while (((encL_overshootCount < numOvershoot) || (encL_count <= remainderCount)) && ((encR_overshootCount < numOvershoot) || (encR_count <= remainderCount)))
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

void rotateLeft_custom(int angle, int tickOffset)
{
//  int tEncodeVal = angle * 4.3; //angle * 4.3; // 4.33;  // 4.41: 100 RPM
//  int numOvershoot = tEncodeVal / 256;
//  int remainderCount = tEncodeVal % 256;
  if (angle == 90) {
    numOvershoot = 1;
    remainderCount = 139;//target 125: 127;// target 110: 139; //123;
  }
  remainderCount += tickOffset;

  // reset encoder ticks
  resetEnc();
  
  while (((encL_overshootCount < numOvershoot) || (encL_count <= remainderCount)) || ((encR_overshootCount < numOvershoot) || (encR_count <= remainderCount)))
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
