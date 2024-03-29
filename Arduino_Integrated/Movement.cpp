#include "Movement.h"
#include "PID.h"
#include "Sensors.h"
#include "Motors.h"
#include "Alignment.h"

bool emergencyBrakes = false;

bool moveForward(int moveUnits, bool emergencyEnabled, bool crashRecovery = true, unsigned int additionalTicks = 0)
{
  // reset encoder ticks
  resetEnc();

  int numOvershoot = numOvershoot_lut[moveUnits];
  int remainderCount = remainderCount_lut[moveUnits];
  // add additional ticks (if any)
  if (additionalTicks > 0) {
    int initialRemainder = remainderCount;
    remainderCount += additionalTicks;
    if (remainderCount <= initialRemainder) {  // overflow
      ++numOvershoot;
    }
  }

  // check if either motor reached the target number of ticks
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
      if (emergencyEnabled) {
  #ifdef EXPLORATION_MODE
          if (crashRecovery)
            checkForCrashAndRecover(numOvershoot, remainderCount);
          else
            checkForCrash();
  #else
          checkForCrashAndRecover(numOvershoot, remainderCount);
  #endif
      }
    }
  }
  if (!emergencyBrakes)
    md.setBrakes(BRAKE_L, BRAKE_R);
  bool eBraked = emergencyBrakes; // true if emergencyBrakes was triggered
  if (emergencyBrakes) {
    // TODO: perform recovery action?
    //Serial.write("EMERGENCY\n");
    emergencyBrakes = false;
  }
  // reset PID
  resetPIDControllers();

  return eBraked;
}
void moveBackward(int moveUnits)
{
  // reset encoder ticks
  resetEnc();
  int numOvershoot = numOvershoot_lut[moveUnits];
  int remainderCount = remainderCount_lut[moveUnits];
  
  // check if either motor reached the target number of ticks
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
#if targetRpm == TARGETRPM_110
    remainderCount = 136;//target 125: 127;// target 110: 139; //123;
#elif targetRpm == TARGETRPM_120
    remainderCount = 130;
#elif targetRpm == TARGETRPM_125
    remainderCount = 136;
#endif
  }
  else if (angle == 180) {
    //tEncodeVal = 810; //angle * 4.5;  // 4.65
    numOvershoot = 3;
#if targetRpm == TARGETRPM_110
    remainderCount = 41;
#elif targetRpm == TARGETRPM_120
  remainderCount = 39;
#elif targetRpm == TARGETRPM_125
    remainderCount = 41;
#endif
  }

//#ifndef EXPLORATION_MODE
//  remainderCount += 5;
//#endif

  // reset encoder ticks
  resetEnc();
  
  while (((encL_overshootCount < numOvershoot) || (encL_count <= remainderCount)) && ((encR_overshootCount < numOvershoot) || (encR_count <= remainderCount)))
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
#if targetRpm == TARGETRPM_110
    remainderCount = 137;//132;
#elif targetRpm == TARGETRPM_120
    remainderCount = 140;
#elif targetRpm == TARGETRPM_125
    remainderCount = 132;
#endif
  }
  else if (angle == 180) {
    //tEncodeVal = 806; //angle * 4.48;
    numOvershoot = 3;
#if targetRpm == TARGETRPM_110
    remainderCount = 38;
#elif targetRpm == TARGETRPM_120
    remainderCount = 36;
#elif targetRpm == TARGETRPM_125
    remainderCount = 36;
#endif
  }

//#ifndef EXPLORATION_MODE
//  remainderCount += 5;
//#endif
  
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

void checkAlignmentAfterCommand_FP() {
  // align robot to be straight
  checkForTilted();
}

void checkAlignmentAfterMove() {
  
  // align robot to be straight
  checkForTilted();
  delay(50);
  
  // ensure robot is centralised within its grids
  checkCentralise_Front();
  delay(50);
  //if (canCheckCentralise_Sides) {
    checkCentralise_Sides();
    delay(50);
  //}
  //canCheckCentralise_Sides = false;
  
  // align robot to be straight
  if (didCentralise) {
    checkForTilted();
    didCentralise = false;
  }
}

void checkAlignmentAfterRotate() {
  // align robot to be straight
  checkForTilted();
}

// function to calibrate sensors in starting grid
void initialGridCalibration() {

  // 1. rotate left for robot's front to face side wall
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
  {
    alignToLeftWall();
  }
}

// CUSTOM MOVEMENT
void moveForward_custom(double distance, bool emergencyEnabled)
{
  // reset encoder ticks
  resetEnc();
  
  int tEncodeVal = distance / ONEREVDIST * 562.25 - MOVE_OFFTICKS;
  int numOvershoot = tEncodeVal / 256;
  int remainderCount = tEncodeVal % 256;

  // check if either motor reached the target number of ticks
  while (!emergencyBrakes &&
  (((encL_overshootCount < numOvershoot) || (encL_count <= remainderCount)) && ((encR_overshootCount < numOvershoot) || (encR_count <= remainderCount))))
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));

    // read IR sensors here to check for emergency brakes
      if (emergencyEnabled) {
  #ifdef EXPLORATION_MODE
          //checkForCrash();
          checkForCrashAndRecover(numOvershoot, remainderCount);
  #else
          //checkForCrashAndRecover(numOvershoot, remainderCount);
  #endif
      }
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

void moveBackward_custom(double distance) {
  
  // reset encoder ticks
  resetEnc();
  // calculate ticks required for movement
  int tEncodeVal = distance / ONEREVDIST * 562.25 - MOVE_OFFTICKS;
  int numOvershoot = tEncodeVal / 256;
  int remainderCount = tEncodeVal % 256;
  
  // check if either motor reached the target number of ticks
  while (((encL_overshootCount < numOvershoot) || (encL_count <= remainderCount)) || ((encR_overshootCount < numOvershoot) || (encR_count <= remainderCount)))
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

void rotateRight_custom(int angle, int tickOffset)
{
  int numOvershoot = 0;
  int remainderCount = 0;
  if (angle == 90) {
    numOvershoot = 1;
#if targetRpm == TARGETRPM_110
    remainderCount = 137;//132;
#elif targetRpm == TARGETRPM_120
    remainderCount = 138;
#elif targetRpm == TARGETRPM_125
    remainderCount = 138;
#endif
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
  int numOvershoot = 0;
  int remainderCount = 0;
  if (angle == 90) {
    numOvershoot = 1;
#if targetRpm == TARGETRPM_110
    remainderCount = 136;//target 125: 127;// target 110: 139; //123;
#elif targetRpm == TARGETRPM_120
    remainderCount = 129;
#elif targetRpm == TARGETRPM_125
    remainderCount = 129;
#endif
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

// W move forward
bool moveForward_W(int moveUnits, int *currStep)
{
  // reset encoder ticks
  resetEnc();
  // make use of this to move an extra step
  *currStep = 0;

  int numOvershoot = numOvershoot_lut[moveUnits];
  int remainderCount = remainderCount_lut[moveUnits];

  // check if either motor reached the target number of ticks
  while (!emergencyBrakes && 
    (((encL_overshootCount < numOvershoot) || (encL_count <= remainderCount)) && ((encR_overshootCount < numOvershoot) || (encR_count <= remainderCount)))
  )
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));

      // read right sensor
      int currTicks = encL_overshootCount * 256 + encL_count;
      if (currTicks >= total_lut[*currStep]) {
#ifndef IMAGEREC_MODE
        sendRightSensorReadings();
#endif
        ++(*currStep);
      }

      // read IR sensors here to check for emergency brakes
      checkForCrash();
    }
  }
  if (!emergencyBrakes)
    md.setBrakes(BRAKE_L, BRAKE_R);
  bool eBraked = emergencyBrakes; // true if emergencyBrakes was triggered
  emergencyBrakes = false;
  // reset PID
  resetPIDControllers();

  return eBraked;
}
