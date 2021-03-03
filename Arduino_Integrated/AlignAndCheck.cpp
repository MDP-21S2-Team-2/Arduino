#include "SharpIR.h"
#include "Movement.h"
#include "PID.h"
#include "AlignAndCheck.h"
#include "Sensors.h"

// check for crash while robot is moving
void checkForCrashCalibration() {
  double dist_D1 = front_D1.getDistance();
  double dist_D2 = front_D2.getDistance();
  double dist_D3 = front_D3.getDistance();

  // check if robot is too near to any obstacle in front, for emergency stop
  if ((/*dist_D1 > 3.0 &&*/ dist_D1 < FRONT_1GRID_DIST + 2.0) ||  // offset to account for robot's speed
    (/*dist_D2 > 3.0 &&*/ dist_D2 < FRONT_1GRID_DIST + 2.0) ||
    (/*dist_D3 > 3.0 &&*/ dist_D3 < FRONT_1GRID_DIST + 2.0)) 
    {
      md.setBrakes(BRAKE_L, BRAKE_R);
      emergencyBrakes = true;
      //Serial.write("EMERGENCY ");
      return; // stop checking
    }
//  double dist_S1 = left_S1.getDistance();
//  double dist_S2 = left_S2.getDistance();
  //double dist_LR = right_long.getDistance();  // can use to check if veering too close to a wall on the right?
  
  // check if robot is aligned on the side or might crash the side
//  if (((dist_S1 > 3.0 && dist_S1 <= 10.5 &&  dist_S2 > 3.0 && dist_S2 <= 10.5) ||
//    (dist_S1 > 10.5 && dist_S1 <= 18.0 &&  dist_S2 > 10.5 && dist_S2 <= 18.0)) &&  // check if robot is near the wall
//    (abs(dist_S1 - dist_S2) >= 2.5)) 
//    {  // robot is veering too much to the side, sensors' diff is 1.5cm
//      // recovery action
//      md.setBrakes(BRAKE_L, BRAKE_R);
//      // store current encoder values (before extra ticks are added for recovery action)
//      int encL = encL_count;
//      int encR = encR_count;
//      // get robot to align to left wall
//      alignToLeftWall();
//      delay(10);  // to ensure robot has already braked; TODO: time to delay TBD
//      // reset PID
//      leftPIDController.resetPID();
//      rightPIDController.resetPID();
//      // restore back the encoder counts to recover remaining distance
//      encL_count = encL;
//      encR_count = encR;
//    }
}


// check during initialGridCalibration, as this is more accurate
void checkForAlignmentCalibration_initial() {

  // get distances
  double dist_D1 = front_D1.getDistance();
  double dist_D2 = front_D2.getDistance();
  double dist_D3 = front_D3.getDistance();

// check for ideal distance from obstacle
  // check if too near to the wall in front
  if (dist_D1 < FRONT_INITIAL_EXPECTED_DIST) { // front-mid sensor
    alignBack_Front(SharpIR::D1, false, FRONT_INITIAL_EXPECTED_DIST);
  }
  else if (dist_D2 < FRONT_INITIAL_EXPECTED_DIST) {  // front-right sensor
    alignBack_Front(SharpIR::D2, false, FRONT_INITIAL_EXPECTED_DIST);
  }
  else if (dist_D3 < FRONT_INITIAL_EXPECTED_DIST) {  // front-left sensor
    alignBack_Front(SharpIR::D3, false, FRONT_INITIAL_EXPECTED_DIST);
  }

  // update distances
  delay(100);
  dist_D1 = front_D1.getDistance();
  dist_D2 = front_D2.getDistance();
  dist_D3 = front_D3.getDistance();
  
  // check if too far from the wall in front
  if (dist_D1 > FRONT_INITIAL_EXPECTED_DIST) { // front-mid sensor
    alignForward_Front(SharpIR::D1, false, FRONT_INITIAL_EXPECTED_DIST);
  }
  else if (dist_D2 > FRONT_INITIAL_EXPECTED_DIST) {  // front-right sensor
    alignForward_Front(SharpIR::D2, false, FRONT_INITIAL_EXPECTED_DIST);
  }
  else if (dist_D3 > FRONT_INITIAL_EXPECTED_DIST) {  // front-left sensor
    alignForward_Front(SharpIR::D3, false, FRONT_INITIAL_EXPECTED_DIST);
  }

  // update distances
  delay(100);
  dist_D1 = front_D1.getDistance();
  dist_D2 = front_D2.getDistance();
  dist_D3 = front_D3.getDistance();
  
// align robot to be straight
  // front right
  if (((dist_D1 >= 3.0 && dist_D1 <= 8.0 && dist_D2 >= 3.0 && dist_D2 <= 8.0) ||
    (dist_D1 >= 8.0 && dist_D1 <= 18.0 && dist_D2 >= 8.0 && dist_D2 <= 18.0) ||
    (dist_D1 >= 18.0 && dist_D1 <= 28.0 && dist_D2 >= 18.0 && dist_D2 <= 28.0))  // only try to align when within certain range from obstacle in front
   && abs(dist_D1 - dist_D2) > 0.2) { // TODO: distance difference threshold
    alignToFrontWall_Right();
    }
  // front left
  else if (((dist_D1 >= 3.0 && dist_D1 <= 8.0 && dist_D3 >= 3.0 && dist_D3 <= 8.0) ||
    (dist_D1 >= 8.0 && dist_D1 <= 18.0 && dist_D3 >= 8.0 && dist_D3 <= 18.0) ||
    (dist_D1 >= 18.0 && dist_D1 <= 28.0 && dist_D3 >= 18.0 && dist_D3 <= 28.0))  // only try to align when within certain range from obstacle in front
   && abs(dist_D1 - dist_D3) > 0.2) { // TODO: distance difference threshold
      alignToFrontWall_Left();
    }

  // TESTING: delay
  delay(500);
}

// check when robot is not moving, e.g. after moving forward, after rotating
void checkForAlignmentCalibration() {

  delay(100); // allow robot to settle
  
  double dist_D1, dist_D2, dist_D3, dist_S1, dist_S2, dist_LR;

// check for ideal distance from obstacle
  // check if too near to the wall right in front
  dist_D1 = front_D1.getDistance();
  dist_D2 = front_D2.getDistance();
  dist_D3 = front_D3.getDistance();
  if (dist_D1 < FRONT_1GRID_DIST) { // front-mid sensor
    alignBack_Front(SharpIR::D1, true, FRONT_1GRID_DIST);
  }
  else if (dist_D2 < FRONT_1GRID_DIST) {  // front-right sensor
    alignBack_Front(SharpIR::D2, true, FRONT_1GRID_DIST);
  }
  else if (dist_D3 < FRONT_1GRID_DIST) {  // front-left sensor
    alignBack_Front(SharpIR::D3, true, FRONT_1GRID_DIST);
  }

  // check if too far from wall right in front
  dist_D1 = front_D1.getDistance();
  dist_D2 = front_D2.getDistance();
  dist_D3 = front_D3.getDistance();
  
  if (dist_D1 > FRONT_1GRID_START && dist_D1 < FRONT_1GRID_END) { // front-mid sensor
    alignForward_Front(SharpIR::D1, true, FRONT_1GRID_DIST);
  }
  else if (dist_D2 > FRONT_1GRID_START && dist_D2 < FRONT_1GRID_END) {  // front-right sensor
    alignForward_Front(SharpIR::D2, true, FRONT_1GRID_DIST);
  }
  else if (dist_D3 > FRONT_1GRID_START && dist_D3 < FRONT_1GRID_END) {  // front-left sensor
    alignForward_Front(SharpIR::D3, true, FRONT_1GRID_DIST);
  }

//check for ideal distance in middle of 3x3 grid - only if left-side calibration not used
  bool alignedWithLeft = false;
  // check if too near to the wall on left side
  dist_S1 = left_S1.getDistance();
  dist_S2 = left_S2.getDistance();
  if (dist_S1 < LEFT_1GRID_DIST - FLUC_THRESHOLD) { // left-front sensor
    rotateLeft(90);
    delay(150);
    alignBack_Front(SharpIR::D2, true, FRONT_1GRID_DIST);  // align with right sensor
    delay(100);
    rotateRight(90);
    delay(150);

    alignedWithLeft = true;
  }
  else if (dist_S2 < LEFT_1GRID_DIST - FLUC_THRESHOLD) {  // left-back sensor
    rotateLeft(90);
    delay(150);
    alignBack_Front(SharpIR::D1, true, FRONT_1GRID_DIST);  // align with middle sensor
    delay(100);
    rotateRight(90);
    delay(150);

    alignedWithLeft = true;
  }
  // check if too far from wall on left side
  else if (dist_S1 > LEFT_1GRID_DIST + FLUC_THRESHOLD) { // left-front sensor
    rotateLeft(90);
    delay(150);
    alignForward_Front(SharpIR::D2, true, FRONT_1GRID_DIST);  // align with right sensor
    delay(100);
    rotateRight(90);
    delay(150);

    alignedWithLeft = true;
  }
  else if (dist_S2 > LEFT_1GRID_DIST + FLUC_THRESHOLD) {  // left-back sensor
    rotateLeft(90);
    delay(150);
    alignForward_Front(SharpIR::D1, true, FRONT_1GRID_DIST);  // align with middle sensor
    delay(100);
    rotateRight(90);
    delay(150);

    alignedWithLeft = true;
  }
  
  // check if too near to the wall on right side
  if (!alignedWithLeft) {
    dist_LR = right_long.getDistance();
    if (dist_LR < RIGHT_1GRID_DIST) { // left-front sensor
      rotateRight(90);
      delay(150);
      alignBack_Front(SharpIR::D3, true, FRONT_1GRID_DIST);  // align with left sensor
      delay(100);
      rotateLeft(90);
      delay(150);
    }
    // check if too far from wall on right side
    else if (dist_LR < RIGHT_1GRID_END) { // left-front sensor
      rotateRight(90);
      delay(150);
      alignForward_Front(SharpIR::D3, true, FRONT_1GRID_DIST);  // align with left sensor
      delay(100);
      rotateLeft(90);
      delay(150);
    }
  }
  
// align robot to be straight
  // if front sensors can be used, use them
  dist_D1 = front_D1.getDistance();
  dist_D2 = front_D2.getDistance();
  dist_D3 = front_D3.getDistance();
  if ((dist_D1 >= 2.0 && dist_D1 <= 28.0 && dist_D2 >= 2.0 && dist_D2 <= 28.0) ||
    (dist_D1 >= 2.0 && dist_D1 <= 28.0 && dist_D3 >= 2.0 && dist_D2 <= 28.0))
  {
    // front right
    if ((dist_D1 >= 1.0 && dist_D1 <= 8.0 && dist_D2 >= 1.0 && dist_D2 <= 8.0 && abs(dist_D1 - dist_D2) > 0.3) ||
      (dist_D1 >= 13.0 && dist_D1 <= 18.0 && dist_D2 >= 13.0 && dist_D2 <= 18.0 && abs(dist_D1 - dist_D2) > 0.5) ||
      (dist_D1 >= 23.0 && dist_D1 <= 28.0 && dist_D2 >= 23.0 && dist_D2 <= 28.0 && abs(dist_D1 - dist_D2) > 0.8))  // only try to align when within certain range from obstacle in front
    { // TODO: distance difference thresholds
      alignToFrontWall_Right();
    }
    // front left
    else if ((dist_D1 >= 1.0 && dist_D1 <= 8.0 && dist_D3 >= 1.0 && dist_D3 <= 8.0 && abs(dist_D1 - dist_D3) > 0.2) ||
      (dist_D1 >= 13.0 && dist_D1 <= 18.0 && dist_D3 >= 13.0 && dist_D3 <= 18.0 && abs(dist_D1 - dist_D3) > 0.5) ||
      (dist_D1 >= 23.0 && dist_D1 <= 28.0 && dist_D3 >= 23.0 && dist_D3 <= 28.0 && abs(dist_D1 - dist_D3) > 0.8))  // only try to align when within certain range from obstacle in front
    { // TODO: distance difference thresholds
        alignToFrontWall_Left();
    }
  } // else, check if left side can be used for alignment
  else {
    dist_S1 = left_S1.getDistance();
    dist_S2 = left_S2.getDistance();
    if ((dist_S1 >= 2.0 && dist_S1 <= 8.0 && dist_S2 >= 3.0 && dist_S2 <= 8.0 && abs(dist_S1 - dist_S2) > 0.1) ||
    (dist_S1 >= 13.0 && dist_S1 <= 18.0 && dist_S2 >= 13.0 && dist_S2 <= 18.0 && abs(dist_S1 - dist_S2) > 0.5) ||
    (dist_S1 >= 23.0 && dist_S1 <= 28.0 && dist_S2 >= 23.0 && dist_S2 <= 28.0 && abs(dist_S1 - dist_S2) > 1.5))
    { // TODO: distance difference threshold
      alignToLeftWall();
    }
  }
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

    difference = dist_S2 - dist_S1;

    // abs: is a macro, so should be efficient
    if (0.075 <=difference && difference <= SIDE_THRESHOLD) { // very small gap, so stop moving
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
      newState = (difference < 0.075) ? STATE_DIFFGT0 : STATE_DIFFLT0;
      if (currState != newState) {  // change in state
        // set new speed
        if (abs(difference) > HS_THRESHOLD) {
          if (newState == STATE_DIFFGT0)  md.setSpeeds(100, 100); // tilted to the right; turn left
          else  md.setSpeeds(-100, -100); // tilted to the left; turn right
        } else {
          if (newState == STATE_DIFFGT0)  md.setSpeeds(60, 60); // tilted to the right; turn left
          else  md.setSpeeds(-60, -60); // tilted to the left; turn right
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
    }
    else {
      newState = (difference > 0) ? STATE_DIFFGT0 : STATE_DIFFLT0;
      if (currState != newState) {  // change in state
        // set new speed
        if (abs(difference) > HS_THRESHOLD) {
          if (newState == STATE_DIFFGT0)  md.setSpeeds(100, 100); // tilted to the right; turn left
          else  md.setSpeeds(-100, -100); // tilted to the left; turn right
        } else {
          if (newState == STATE_DIFFGT0)  md.setSpeeds(60, 60); // tilted to the right; turn left
          else  md.setSpeeds(-60, -60); // tilted to the left; turn right
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
    }
    else {
      newState = (difference > 0) ? STATE_DIFFGT0 : STATE_DIFFLT0;
      if (currState != newState) {  // change in state
        // set new speed
        if (abs(difference) > HS_THRESHOLD) {
          if (newState == STATE_DIFFGT0)  
            md.setSpeeds(100, 100); // tilted to the right; turn left
          else  
            md.setSpeeds(-100, -100); // tilted to the left; turn right
        } else {
          if (newState == STATE_DIFFGT0)  
            md.setSpeeds(60, 60); // tilted to the right; turn left
          else  
            md.setSpeeds(-60, -60); // tilted to the left; turn right
        }
        currState = newState;
      }
    }
  }
}

// Robot self-calibration: repositioning
void alignBack_Front(SharpIR::sensorCode sensor, bool fast, double targetDist) {
  
  // use one of the front sensors for alignment
  double dist_Dx = 0.0;

  //if (fast)
    md.setSpeeds(80, -80);
  //else
  //  md.setSpeeds(60, -60);
  
  do {
    switch (sensor) {
    case SharpIR::D1: dist_Dx = front_D1.getDistance();
      break;
    case SharpIR::D2: dist_Dx = front_D2.getDistance();
      break;
    case SharpIR::D3: dist_Dx = front_D3.getDistance();
      break;
    }
  } while (dist_Dx < targetDist && dist_Dx > targetDist - 5);
  md.setBrakes(BRAKE_L, BRAKE_R);
}

void alignForward_Front(SharpIR::sensorCode sensor, bool fast, double targetDist) {
  
  // use one of the front sensors for alignment
  double dist_Dx = 0.0;

  //if (fast)
    md.setSpeeds(-80, 80);
  //else
  //  md.setSpeeds(-60, 60);
  
  do {
    switch (sensor) {
    case SharpIR::D1: dist_Dx = front_D1.getDistance();
      break;
    case SharpIR::D2: dist_Dx = front_D2.getDistance();
      break;
    case SharpIR::D3: dist_Dx = front_D3.getDistance();
      break;
    }
  } while (dist_Dx > targetDist && dist_Dx < targetDist + 5);
  md.setBrakes(BRAKE_L, BRAKE_R);
}

// function to calibrate sensors in starting grid
void initialGridCalibration() {

  // 1. rotate left for robot's front to face wall
  rotateLeft(90);
  
  // 3. TBD: wait a short while (for brakes to settle?)
  delay(500);
  
  // 2. align accurately with wall in front
  checkForAlignmentCalibration_initial();

  // 3. TBD: wait a short while (for brakes to settle)
  delay(500);

  // 4. turn left again to face wall behind
  rotateLeft(90);

  // 5. align with wall so robot is correct distance away
  if (front_D1.getDistance() < 4) { // front-mid sensor
    alignBack_Front(SharpIR::D1, false, 4.0);
  }
  else if (front_D1.getDistance() > 4.5) {  // front-right sensor
    alignForward_Front(SharpIR::D2, false, 4.0);
  }

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
