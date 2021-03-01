#include "SharpIR.h"
#include "Movement.h"
#include "PID.h"
#include "AlignAndCheck.h"
#include "Sensors.h"

double D1_EXPECTED_DIST = 4.0; 
double D2_EXPECTED_DIST = 4.0;
double D3_EXPECTED_DIST = 4.0;
double S1_EXPECTED_DIST = 6.0;
double S2_EXPECTED_DIST = 6.0;

// check for crash while robot is moving
void checkForCrashCalibration() {
  double dist_D1 = front_D1.getDistance();
  double dist_D2 = front_D2.getDistance();
  double dist_D3 = front_D3.getDistance();

  // check if robot is too near to any obstacle in front, for emergency stop
  if ((/*dist_D1 > 3.0 &&*/ dist_D1 < D1_EXPECTED_DIST + 3.0) ||  // offset to account for robot's speed
    (/*dist_D2 > 3.0 &&*/ dist_D2 < D2_EXPECTED_DIST + 3.0) ||
    (/*dist_D3 > 3.0 &&*/ dist_D3 < D3_EXPECTED_DIST + 3.0)) 
    {
      md.setBrakes(BRAKE_L, BRAKE_R);
      emergencyBrakes = true;
      Serial.write("EMERGENCY ");
      return; // stop checking
    }
  double dist_S1 = left_S1.getDistance();
  double dist_S2 = left_S2.getDistance();
  //double dist_LR = right_long.getDistance();  // can use to check if veering too close to a wall on the right?
  
  // check if robot is aligned on the side or might crash the side
  if ((dist_S1 > 3.0 && dist_S1 < 23.0) &&  // check if robot is near the wall
    (dist_S2 > 3.0 && dist_S2 < 23.0) &&    // check if robot is near the wall
    (abs(dist_S1 - dist_S2) >= 2.3)) 
    {  // robot is veering too much to the side, sensors' diff is 1.5cm
      // recovery action
      md.setBrakes(BRAKE_L, BRAKE_R);
      // store current encoder values (before extra ticks are added for recovery action)
      int encL = encL_count;
      int encR = encR_count;
      // get robot to align to left wall
      alignToLeftWall();
      delay(10);  // to ensure robot has already braked; TODO: time to delay TBD
      // reset PID
      leftPIDController.resetPID();
      rightPIDController.resetPID();
      // restore back the encoder counts to recover remaining distance
      encL_count = encL;
      encR_count = encR;
    }
}


// check during initialGridCalibration, as this is more accurate
void checkForAlignmentCalibration_initial() {

  // TESTING: check distances
  double dist_D1 = front_D1.getDistance();
  double dist_D2 = front_D2.getDistance();
  double dist_D3 = front_D3.getDistance();
//  double dist_S1 = left_S1.getDistance();
//  double dist_S2 = left_S2.getDistance();
//  double dist_LR = right_long.getDistance();
  Serial.print(dist_D1);
  Serial.write(",");
  Serial.print(dist_D2);
  Serial.write(",");
  Serial.print(dist_D3);
  Serial.write("\n");

// check for ideal distance from obstacle
  // check if too near to the wall in front
  if (dist_D1 < FRONT_INITIAL_EXPECTED_DIST) { // front-mid sensor
    alignBack_Front(SharpIR::D1, false);
  }
  else if (dist_D2 < FRONT_INITIAL_EXPECTED_DIST) {  // front-right sensor
    alignBack_Front(SharpIR::D2, false);
  }
  else if (dist_D3 < FRONT_INITIAL_EXPECTED_DIST) {  // front-left sensor
    alignBack_Front(SharpIR::D3, false);
  }

  // TESTING: check distances
  dist_D1 = front_D1.getDistance();
  dist_D2 = front_D2.getDistance();
  dist_D3 = front_D3.getDistance();
  Serial.print(dist_D1);
  Serial.write(",");
  Serial.print(dist_D2);
  Serial.write(",");
  Serial.print(dist_D3);
  Serial.write("\n");
  
  // check if too far from the wall in front
  if (dist_D1 > FRONT_INITIAL_EXPECTED_DIST) { // front-mid sensor
    alignForward_Front(SharpIR::D1, false);
  }
  else if (dist_D2 > FRONT_INITIAL_EXPECTED_DIST) {  // front-right sensor
    alignForward_Front(SharpIR::D2, false);
  }
  else if (dist_D3 > FRONT_INITIAL_EXPECTED_DIST) {  // front-left sensor
    alignForward_Front(SharpIR::D3, false);
  }

  // TESTING: check distances
  dist_D1 = front_D1.getDistance();
  dist_D2 = front_D2.getDistance();
  dist_D3 = front_D3.getDistance();
  Serial.print(dist_D1);
  Serial.write(",");
  Serial.print(dist_D2);
  Serial.write(",");
  Serial.print(dist_D3);
  Serial.write("\n");
  
// align robot to be straight
  // front right
  if (dist_D1 >= 3.0 && dist_D1 <= 33.0 && dist_D2 >= 3.0 && dist_D2 <= 33.0  // only try to align when within certain range from obstacle in front
   && abs(dist_D1 - dist_D2) > 0.2) 
    { // TODO: distance difference threshold
    alignToFrontWall_Right();
    }
  // front left
  else if (dist_D1 >= 3.0 && dist_D1 <= 23.0 && dist_D3 >= 3.0 && dist_D3 <= 23.0  // only try to align when within certain range from obstacle in front
   && abs(dist_D1 - dist_D3) > 0.2) 
    { // TODO: distance difference threshold
      alignToFrontWall_Left();
    }

  // TESTING: delay
  delay(1000);
  // TESTING: check distances
  dist_D1 = front_D1.getDistance();
  dist_D2 = front_D2.getDistance();
  dist_D3 = front_D3.getDistance();
  Serial.print(dist_D1);
  Serial.write(",");
  Serial.print(dist_D2);
  Serial.write(",");
  Serial.print(dist_D3);
  Serial.write("\n");
}

// check when robot is not moving, e.g. after moving forward, after rotating
void checkForAlignmentCalibration() {
  
  double dist_D1 = front_D1.getDistance();
  double dist_D2 = front_D2.getDistance();
  double dist_D3 = front_D3.getDistance();
  double dist_S1 = left_S1.getDistance();
  double dist_S2 = left_S2.getDistance();
  double dist_LR = right_long.getDistance();

// check for ideal distance from obstacle
  // check if too near to the wall in front
  if (dist_D1 < D1_EXPECTED_DIST) { // front-mid sensor
    alignBack_Front(SharpIR::D1, true);
  }
  else if (dist_D2 < D2_EXPECTED_DIST) {  // front-right sensor
    alignBack_Front(SharpIR::D2, true);
  }
  else if (dist_D3 < D3_EXPECTED_DIST) {  // front-left sensor
    alignBack_Front(SharpIR::D3, true);
  }
  
// align robot to be straight
  // front right
  if (dist_D1 >= 3.0 && dist_D1 <= 33.0 && dist_D2 >= 3.0 && dist_D2 <= 33.0  // only try to align when within certain range from obstacle in front
   && abs(dist_D1 - dist_D2) > 0.8) { // TODO: distance difference threshold
    alignToFrontWall_Right();
  }
  // front left
  else if (dist_D1 >= 3.0 && dist_D1 <= 23.0 && dist_D3 >= 3.0 && dist_D3 <= 23.0  // only try to align when within certain range from obstacle in front
   && abs(dist_D1 - dist_D3) > 0.8) { // TODO: distance difference threshold
    alignToFrontWall_Left();
  }
  // left side
  else if (dist_S1 >= 3.0 && dist_S1 <= 23.0 && dist_S2 >= 3.0 && dist_S2 <= 23.0
   && abs(dist_S1 - dist_S2) > 1.5) { // TODO: distance difference threshold
    alignToLeftWall();
  }
}


void alignBack_FrontMid() {
  
  // use side sensors for alignment
  double dist_D1 = 2.0;
  md.setSpeeds(100, -100);
  while (dist_D1 < 9.0) {
    dist_D1 = front_D1.getDistance();
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
}

// Robot self-calibration: wall alignment
void alignToLeftWall() {
  
  // use side sensors for alignment
  double dist_S1, dist_S2, difference;
  bool checkAlignment = true;

  // 0: have not started moving; 1: diff > 0; 2: diff < 0
  char currState = 0, newState = 0;
  
  while (checkAlignment) {
    dist_S1 = left_S1.getDistance();
    dist_S2 = left_S2.getDistance();

    difference = dist_S1 - dist_S2;

    // abs: is a macro, so should be efficient
    if (abs(difference) <= THRESHOLD) { // very small gap, so stop moving
      //delay(1); // 1ms
      md.setBrakes(BRAKE_L, BRAKE_R);
      checkAlignment = false;
    } /*else if (abs(difference) <= THRESHOLD_1) {  // continue aligning at a decreasing speed
      //motorSpeed -= 10;
      if (difference > 0) { // tilted to the right; turn left
        md.setSpeeds(20, 20);
      } else {  // tilted to the left; turn right
        md.setSpeeds(-20, -20);
      }
    } // greater than first threshold
    */
    else {
      newState = (difference > 0) ? STATE_DIFFGT0 : STATE_DIFFLT0;
      if (currState != newState) {  // change in state
        // set new speed
        if (abs(difference) > HS_THRESHOLD) {
          if (newState == STATE_DIFFGT0)  md.setSpeeds(100, 100); // tilted to the right; turn left
          else  md.setSpeeds(-100, -100); // tilted to the left; turn right
        } else {
          if (newState == STATE_DIFFGT0)  md.setSpeeds(70, 70); // tilted to the right; turn left
          else  md.setSpeeds(-70, -70); // tilted to the left; turn right
        }
        currState = newState;
      }
    }
  }
}

void alignToFrontWall_Left() {
  // use front-left (D3) and front-mid (D1) for alignment
  double dist_D1, dist_D3, difference;
  bool checkAlignment = true;

  // 0: have not started moving; 1: diff > 0; 2: diff < 0
  char currState = 0, newState = 0;
  
  while (checkAlignment) {
    dist_D1 = front_D1.getDistance();
    dist_D3 = front_D3.getDistance();

    difference = dist_D1 - dist_D3;

    // abs: is a macro, so should be efficient
    if (abs(difference) <= THRESHOLD) { // very small gap, so stop moving
      //delay(1); // 1ms
      md.setBrakes(BRAKE_L, BRAKE_R);
      checkAlignment = false;
    } /*else if (abs(difference) <= THRESHOLD_1) {  // continue aligning at a decreasing speed
      //motorSpeed -= 10;
      if (difference > 0) { // tilted to the right; turn left
        md.setSpeeds(20, 20);
      } else {  // tilted to the left; turn right
        md.setSpeeds(-20, -20);
      }
    } // greater than first threshold
    */
    else {
      newState = (difference > 0) ? STATE_DIFFGT0 : STATE_DIFFLT0;
      if (currState != newState) {  // change in state
        // set new speed
        if (abs(difference) > HS_THRESHOLD) {
          if (newState == STATE_DIFFGT0)  md.setSpeeds(100, 100); // tilted to the right; turn left
          else  md.setSpeeds(-100, -100); // tilted to the left; turn right
        } else {
          if (newState == STATE_DIFFGT0)  md.setSpeeds(70, 70); // tilted to the right; turn left
          else  md.setSpeeds(-70, -70); // tilted to the left; turn right
        }
        currState = newState;
      }
    }
  }
}

void alignToFrontWall_Right() {
  // use front-right (D2) and front-mid (D1) for alignment
  double dist_D1, dist_D2, difference;
  bool checkAlignment = true;

  // 0: have not started moving; 1: diff > 0; 2: diff < 0
  char currState = 0, newState = 0;
  
  while (checkAlignment) {
    dist_D1 = front_D1.getDistance();
    dist_D2 = front_D2.getDistance();

    difference = dist_D2 - dist_D1;

    // abs: is a macro, so should be efficient
    if (abs(difference) <= THRESHOLD) { // very small gap, so stop moving
      //delay(1); // 1ms
      md.setBrakes(BRAKE_L, BRAKE_R);
      checkAlignment = false;
    } /*else if (abs(difference) <= THRESHOLD_1) {  // continue aligning at a decreasing speed
      //motorSpeed -= 10;
      if (difference > 0) { // tilted to the right; turn left
        md.setSpeeds(20, 20);
      } else {  // tilted to the left; turn right
        md.setSpeeds(-20, -20);
      }
    } // greater than first threshold
    */
    else {
      newState = (difference > 0) ? STATE_DIFFGT0 : STATE_DIFFLT0;
      if (currState != newState) {  // change in state
        // set new speed
        if (abs(difference) > HS_THRESHOLD) {
          if (newState == STATE_DIFFGT0)  md.setSpeeds(100, 100); // tilted to the right; turn left
          else  md.setSpeeds(-100, -100); // tilted to the left; turn right
        } else {
          if (newState == STATE_DIFFGT0)  md.setSpeeds(70, 70); // tilted to the right; turn left
          else  md.setSpeeds(-70, -70); // tilted to the left; turn right
        }
        currState = newState;
      }
    }
  }
}

// Robot self-calibration: repositioning
void alignBack_Front(SharpIR::sensorCode sensor, bool fast) {
  
  // use one of the front sensors for alignment
  double dist_Dx = 2.0;

  if (fast)
    md.setSpeeds(80, -80);
  else
    md.setSpeeds(60, -60);
  
  while (dist_Dx < 4.0) {
    switch (sensor) {
    case SharpIR::D1: dist_Dx = front_D1.getDistance();
      break;
    case SharpIR::D2: dist_Dx = front_D2.getDistance();
      break;
    case SharpIR::D3: dist_Dx = front_D3.getDistance();
      break;
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
}

void alignForward_Front(SharpIR::sensorCode sensor, bool fast) {
  
  // use one of the front sensors for alignment
  double dist_Dx = 6.0;

  if (fast)
    md.setSpeeds(-80, 80);
  else
    md.setSpeeds(-60, 60);
  
  while (dist_Dx > 4.0) {
    switch (sensor) {
    case SharpIR::D1: dist_Dx = front_D1.getDistance();
      break;
    case SharpIR::D2: dist_Dx = front_D2.getDistance();
      break;
    case SharpIR::D3: dist_Dx = front_D3.getDistance();
      break;
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
}

// function to calibrate sensors in starting grid
void initialGridCalibration() {

  // 1. rotate left for robot's front to face wall
  rotateLeft(90);
  
  // 3. TBD: wait a short while (for brakes to settle?)
  delay(1000);
  
  // 2. align accurately with wall in front
  checkForAlignmentCalibration_initial();

  // 3. TBD: wait a short while (for brakes to settle)
  delay(1000);
  
  // 3. get readings for left-side sensors
  // idea: get the median reading from multiple sensor reads?
  S1_EXPECTED_DIST = left_S1.getDistance();
  S2_EXPECTED_DIST = left_S2.getDistance();
  
  // 4. get readings for front sensors
  // idea: get the median reading from multiple sensor reads?
  D1_EXPECTED_DIST = front_D1.getDistance();
  D2_EXPECTED_DIST = front_D2.getDistance();
  D3_EXPECTED_DIST = front_D3.getDistance();

  // 4. rotate back to face the front
  rotateRight(90);

  // 5. TBD: align with side sensors?

  // TBD: if sensor readings are too different, do a double-check?

}
