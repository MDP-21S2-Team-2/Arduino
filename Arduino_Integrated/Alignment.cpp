#include "SharpIR.h"
#include "Movement.h"
#include "Alignment.h"
#include "Sensors.h"
#include "Motors.h"

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
  if (dist_S1 < LEFT_1GRID_DIST - SIDES_DIST_THRESHOLD) { // left-front sensor
//    rotateLeft(90);
//    delay(500);
//    alignBack_Front(SharpIR::D2, true, LEFT_1GRID_DIST);  // align with right sensor
//    delay(300);
//    rotateRight(90);
//    delay(500);

    Serial.println("Dist_S1 < LEFT1GRID +");

    alignedWithLeft = true;
  }
  else if (dist_S2 < LEFT_1GRID_DIST - SIDES_DIST_THRESHOLD) {  // left-back sensor
//    rotateLeft(90);
//    delay(500);
//    alignBack_Front(SharpIR::D1, true, LEFT_1GRID_DIST);  // align with middle sensor
//    delay(300);
//    rotateRight(90);
//    delay(500);

    Serial.println("Dist_S2 < LEFT1GRID + ");
    
    alignedWithLeft = true;
  }
  // check if too far from wall on left side
  else if (dist_S1 > LEFT_1GRID_DIST + SIDES_DIST_THRESHOLD) { // left-front sensor
//    rotateLeft(90);
//    delay(500);
//    alignForward_Front(SharpIR::D2, true, LEFT_1GRID_DIST);  // align with right sensor
//    delay(300);
//    rotateRight(90);
//    delay(500);

    Serial.println("Dist_S1 > LEFT1GRID + ");

    alignedWithLeft = true;
  }
  else if (dist_S2 > LEFT_1GRID_DIST + SIDES_DIST_THRESHOLD) {  // left-back sensor
//    rotateLeft(90);
//    delay(300);
//    alignForward_Front(SharpIR::D1, true, LEFT_1GRID_DIST);  // align with middle sensor
//    delay(300);
//    rotateRight(90);
//    delay(500);

    Serial.println("Dist_S2 > LEFT1GRID");

    alignedWithLeft = true;
  }
  
  // check if too near to the wall on right side - only if left-side calibration not used
  if (!alignedWithLeft) {
    dist_LR = right_long.getDistance();
    if (dist_LR < RIGHT_1GRID_DIST - SIDES_DIST_THRESHOLD) { // left-front sensor
//      rotateRight(90);
//      delay(500);
//      alignBack_Front(SharpIR::D3, true, RIGHT_1GRID_DIST);  // align with left sensor
//      delay(300);
//      rotateLeft(90);
//      delay(500);

      Serial.println("Dist_LR < RIGHT1GRID");
    }
    // check if too far from wall on right side
    else if (dist_LR > RIGHT_1GRID_DIST + SIDES_DIST_THRESHOLD) { // left-front sensor
//      rotateRight(90);
//      delay(500);
//      alignForward_Front(SharpIR::D3, true, RIGHT_1GRID_DIST);  // align with left sensor
//      delay(300);
//      rotateLeft(90);
//      delay(500);

      Serial.println("Dist_LR > RIGHT1GRID_END");
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
}

void alignToFrontWall(bool useLeft) {
  // use front-left (D3) and front-mid (D1) for alignment
  double dist_D1, dist_Dx, difference;
  bool checkAlignment = true;

  // 0: have not started moving; 1: diff > 0; 2: diff < 0
  char currState = 0, newState = 0;
  
  while (checkAlignment) {
    dist_D1 = front_D1.getDistance();
    dist_Dx = useLeft ? front_D3.getDistance() : front_D2.getDistance();

    difference = useLeft ? dist_D1 - dist_Dx : dist_Dx - dist_D1;

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
