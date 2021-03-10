#include "SharpIR.h"
#include "Movement.h"
#include "Alignment.h"
#include "Sensors.h"
#include "Motors.h"

void checkForCrashAndRecover(int tOvershoot, int tRemainder) {
  double dist_D1 = front_D1.getDistance();
  double dist_D2 = front_D2.getDistance();
  double dist_D3 = front_D3.getDistance();

  // check if robot is too near to any obstacle in front, for emergency stop
  if ((/*dist_D1 > 3.0 &&*/ dist_D1 < FRONT_1GRID_DIST + 2.0) ||  // offset to account for robot's speed
    (/*dist_D2 > 3.0 &&*/ dist_D2 < FRONT_1GRID_DIST + 2.0) ||
    (/*dist_D3 > 3.0 &&*/ dist_D3 < FRONT_1GRID_DIST + 2.0)) 
    {
      md.setBrakes(BRAKE_L, BRAKE_R);

      // recovery action if necessary
      int encL_curr = encL_overshootCount * 256 + encL_count;
      int encR_curr = encR_overshootCount * 256 + encR_count;

      int targetCount = tOvershoot * 256 + tRemainder;

      // check if recovery action is necessary
      // ~271 ticks is 1 unit (10cm), so check if at least around that much distance was not covered yet
      if (targetCount - encL_curr > 250) {  // at least slightly less than 1 unit not covered yet
        // 1. check which sensor blocked
        // get readings again
        dist_D1 = front_D1.getDistance(); // middle
        dist_D2 = front_D2.getDistance(); // right
        dist_D3 = front_D3.getDistance(); // left

        // robot veered to the left; obstacle on the left
        if (dist_D3 < FRONT_1GRID_DIST + 2.5) { // front-left < 7.5
          delay(1000);
          // 2. rotate right by >90
          rotateRight_custom(95);
          //rotateRight(90);

          delay(1000);

          // 3. move forward abit
          if (dist_D1 < FRONT_1GRID_DIST + 2.5) // if middle blocked, move forward abit more
            moveForward_custom(10.0, false);
          else  // just move about half a grid
            moveForward_custom(7.0, false);

          delay(1000);

          // 4. rotate left
          rotateLeft(90);

          delay(1000);

          // 5. move remaining
          int remainingCount = targetCount - encL_curr;
          double dist = oneRevDis * ((double)remainingCount / 562.25);  // in cm
          moveForward_custom(dist, true);
        }
        // robot veered to the right; obstacle on the right
        else if (dist_D2 < FRONT_1GRID_DIST + 2.5) {  // front-right < 7.5
          delay(1000);
          // 2. rotate left by >90
          rotateLeft_custom(95);
          //rotateLeft(90);

          delay(1000);

          // 3. move forward abit
          if (dist_D1 < FRONT_1GRID_DIST + 2.5) // if middle blocked, move forward abit more
            moveForward_custom(10.0, false);
          else  // just move about half a grid
          moveForward_custom(7.0, false);

          delay(1000);

          // 4. rotate right
          rotateRight(90);

          delay(1000);

          // 5. move remaining
          int remainingCount = targetCount - encL_curr;
          double dist = oneRevDis * ((double)remainingCount / 562.25);  // in cm
          moveForward_custom(dist, true);
        }
          
        // edge case: only middle blocked, not handled
        
      }
      emergencyBrakes = true;
      
      return; // stop checking
    }
}

// check for crash while robot is moving
void checkForCrash() {
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
      return; // stop checking
    }
//  double dist_S1 = left_S1.getDistance();
//  double dist_S2 = left_S2.getDistance();
//  //double dist_LR = right_long.getDistance();  // can use to check if veering too close to a wall on the right?
//  
//  // check if robot is aligned on the side or might crash the side
//    double difference = dist_S2 - dist_S1;
//    if (dist_S1 >= 2.0 && dist_S1 <= 8.0 && dist_S2 >= 2.0 && dist_S2 <= 8.0 && (difference > 0.3 || difference < 0))
//    {  // robot is veering too much to the side
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

// check that robot is in the center of the grids
void checkCentralise_Front() {

  double dist_D1, dist_D2, dist_D3;
  
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
  
  if (dist_D1 > (FRONT_1GRID_DIST + FLUC_THRESHOLD) && dist_D1 < FRONT_1GRID_END) { // front-mid sensor
    alignForward_Front(SharpIR::D1, true, FRONT_1GRID_DIST);
  }
  else if (dist_D2 > (FRONT_1GRID_DIST + FLUC_THRESHOLD) && dist_D2 < FRONT_1GRID_END) {  // front-right sensor
    alignForward_Front(SharpIR::D2, true, FRONT_1GRID_DIST);
  }
  else if (dist_D3 > (FRONT_1GRID_DIST + FLUC_THRESHOLD) && dist_D3 < FRONT_1GRID_END) {  // front-left sensor
    alignForward_Front(SharpIR::D3, true, FRONT_1GRID_DIST);
  }
}

void checkCentralise_Sides() {
  
  double dist_S1, dist_S2, dist_LR;

//check for ideal distance in middle of 3x3 grid
  bool alignedWithLeft = false;
  // check if too near to the wall on left side
  dist_S1 = left_S1.getDistance();
  dist_S2 = left_S2.getDistance();
  if (dist_S1 < LEFT_1GRID_DIST - SIDES_DIST_THRESHOLD) { // S1 sensor < 3.75
    rotateLeft(90);
    delay(500);
    alignBack_Front(SharpIR::D2, true, FRONT_1GRID_DIST);  // align with right sensor
    delay(300);
    rotateRight(90);
    delay(500);

    alignedWithLeft = true;
  }
  else if (dist_S2 < LEFT_1GRID_DIST - SIDES_DIST_THRESHOLD) {  // S2 sensor < 3.75
    rotateLeft(90);
    delay(500);
    alignBack_Front(SharpIR::D1, true, FRONT_1GRID_DIST);  // align with middle sensor
    delay(300);
    rotateRight(90);
    delay(500);
    
    alignedWithLeft = true;
  }
  // check if too far from wall on left side
  else if (dist_S1 > LEFT_1GRID_DIST + SIDES_DIST_THRESHOLD && dist_S1 < LEFT_1GRID_END) { // S1 sensor > 5.75
    rotateLeft(90);
    delay(500);
    alignForward_Front(SharpIR::D2, true, FRONT_1GRID_DIST);  // align with right sensor
    delay(300);
    rotateRight(90);
    delay(500);

    alignedWithLeft = true;
  }
  else if (dist_S2 > LEFT_1GRID_DIST + SIDES_DIST_THRESHOLD && dist_S2 < LEFT_1GRID_END) {  // S2 sensor > 5.75
    rotateLeft(90);
    delay(300);
    alignForward_Front(SharpIR::D1, true, FRONT_1GRID_DIST);  // align with middle sensor
    delay(300);
    rotateRight(90);
    delay(500);

    alignedWithLeft = true;
  }
  
  // check if too near to the wall on right side - only if left-side calibration not used
  if (!alignedWithLeft) {
    dist_LR = right_long.getDistance();
    if (dist_LR < RIGHT_1GRID_DIST - LR_SIDES_DIST_THRESHOLD) { // right LR sensor < 3.5
      rotateRight(90);
      delay(500);
      alignBack_Front(SharpIR::D3, true, FRONT_1GRID_DIST);  // align with left sensor
      delay(300);
      rotateLeft(90);
      delay(500);
    }
    // check if too far from wall on right side
    else if (dist_LR > RIGHT_1GRID_DIST + LR_SIDES_DIST_THRESHOLD && dist_LR < RIGHT_1GRID_END) { // right LR sensor > 5.9
      rotateRight(90);
      delay(500);
      alignForward_Front(SharpIR::D3, true, FRONT_1GRID_DIST);  // align with left sensor
      delay(300);
      rotateLeft(90);
      delay(500);
    }
  }
}

// check if robot is tilted, i.e. not straight on the grids
void checkForTilted() {
// if front sensors can be used, use them instead of side
  double dist_D1 = front_D1.getDistance();
  double dist_D2 = front_D2.getDistance();
  double dist_D3 = front_D3.getDistance();
  if ((dist_D1 >= 2.0 && dist_D1 <= 28.0 && dist_D2 >= 2.0 && dist_D2 <= 28.0) ||
    (dist_D1 >= 2.0 && dist_D1 <= 28.0 && dist_D3 >= 2.0 && dist_D2 <= 28.0))
  {
    double difference1 = abs(dist_D1 - dist_D2);
    double difference2 = abs(dist_D1 - dist_D3);
    // front right
    if ((dist_D1 >= 1.0 && dist_D1 <= 8.0 && dist_D2 >= 1.0 && dist_D2 <= 8.0 && difference1 > 0.3) ||
      (dist_D1 >= 13.0 && dist_D1 <= 18.0 && dist_D2 >= 13.0 && dist_D2 <= 18.0 && difference1 > 0.5) ||
      (dist_D1 >= 23.0 && dist_D1 <= 28.0 && dist_D2 >= 23.0 && dist_D2 <= 28.0 && difference1 > 0.8))  // only try to align when within certain range from obstacle in front
    { // TODO: distance difference thresholds
      alignToFrontWall(false);  // align with right
    }
    // front left
    else if ((dist_D1 >= 1.0 && dist_D1 <= 8.0 && dist_D3 >= 1.0 && dist_D3 <= 8.0 && difference2 > 0.2) ||
      (dist_D1 >= 13.0 && dist_D1 <= 18.0 && dist_D3 >= 13.0 && dist_D3 <= 18.0 && difference2 > 0.5) ||
      (dist_D1 >= 23.0 && dist_D1 <= 28.0 && dist_D3 >= 23.0 && dist_D3 <= 28.0 && difference2 > 0.8))  // only try to align when within certain range from obstacle in front
    { // TODO: distance difference thresholds
        alignToFrontWall(true); // align with left
    }
  }
// else, check if left side can be used for alignment
  else {
    double dist_S1 = left_S1.getDistance();
    double dist_S2 = left_S2.getDistance();
    double difference = dist_S2 - dist_S1;
    if ((dist_S1 >= 2.0 && dist_S1 <= 8.0 && dist_S2 >= 2.0 && dist_S2 <= 8.0 && (difference > 0.15 || difference < 0))/* ||
    (dist_S1 >= 13.0 && dist_S1 <= 18.0 && dist_S2 >= 13.0 && dist_S2 <= 18.0 && difference > 0.2) ||
    (dist_S1 >= 23.0 && dist_S1 <= 28.0 && dist_S2 >= 23.0 && dist_S2 <= 28.0 && abs(dist_S1 - dist_S2) > 0.8)*/)
    { // TODO: distance difference threshold
      alignToLeftWall();
    }
  }
}

// Robot self-calibration: wall alignment
void alignToLeftWall() {
  
  // use side sensors for alignment
  double dist_S1, dist_S2, difference;

  // 0: have not started moving; 1: diff > 0; 2: diff < 0
  char currState = 0, newState = 0;

  unsigned long startTime = millis();
  
  while ((millis() - startTime) < 500) {
    dist_S1 = left_S1.getDistance();
    dist_S2 = left_S2.getDistance();

    difference = dist_S2 - dist_S1;

    // abs: is a macro, so should be efficient
    if (0.075 <=difference && difference <= SIDE_THRESHOLD) { // very small gap, so stop moving
      //delay(1); // 1ms
      md.setBrakes(BRAKE_L, BRAKE_R);
      break;
    }
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
  md.setBrakes(BRAKE_L, BRAKE_R);
}

void alignToFrontWall(bool useLeft) {
  // use front-left (D3) and front-mid (D1) for alignment
  double dist_D1, dist_Dx, difference;

  // 0: have not started moving; 1: diff > 0; 2: diff < 0
  char currState = 0, newState = 0;
  
  unsigned long startTime = millis();
  
  while ((millis() - startTime) < 500) {
    dist_D1 = front_D1.getDistance();
    dist_Dx = useLeft ? front_D3.getDistance() : front_D2.getDistance();

    difference = useLeft ? dist_D1 - dist_Dx : dist_Dx - dist_D1;

    // abs: is a macro, so should be efficient
    if (abs(difference) <= THRESHOLD) { // very small gap, so stop moving
      //delay(1); // 1ms
      md.setBrakes(BRAKE_L, BRAKE_R);
      break;
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
  md.setBrakes(BRAKE_L, BRAKE_R);
}

// Robot self-calibration: repositioning
void alignBack_Front(SharpIR::sensorCode sensor, bool fast, double targetDist) {
  
  // use one of the front sensors for alignment
  double dist_Dx = 0.0;

//  if (fast)
    md.setSpeeds(80, -80);
//  else
//    md.setSpeeds(60, -60);
  
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

//  if (fast)
    md.setSpeeds(-80, 80);
//  else
//    md.setSpeeds(-60, 60);
  
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
