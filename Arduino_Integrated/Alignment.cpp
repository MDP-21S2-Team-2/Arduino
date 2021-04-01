#include "SharpIR.h"
#include "Movement.h"
#include "Alignment.h"
#include "Sensors.h"
#include "Motors.h"

//bool canCheckCentralise_Sides = false;
bool didCentralise = false;

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
      if (targetCount - encL_curr > 200) {  // at least slightly less than 1 unit not covered yet
        // 1. check which sensor blocked
        delay(120);
        // get readings again
        dist_D1 = front_D1.getDistance(); // middle
        dist_D2 = front_D2.getDistance(); // right
        dist_D3 = front_D3.getDistance(); // left

        // robot veered to the left; obstacle on the left
        if (dist_D3 < FRONT_1GRID_DIST + 2.5 && dist_D2 > FRONT_1GRID_END+1.0) { // front-left < 7.5; front-right not blocked
          // 2. rotate right by >90
          rotateRight_custom(90, 12);
          //rotateRight(90);

          delay(80);

          // 3. move forward abit
          if (dist_D1 < FRONT_1GRID_DIST + 2.5) // if middle was also blocked, move forward abit more
            moveForward_custom(10.5, true);
          else  // just move about half a grid
            moveForward_custom(7.0, true);

          delay(80);

          // 4. rotate left
          rotateLeft(90);

          delay(80);

          // 5. move remaining
          int remainingCount = targetCount - encL_curr;
          double dist = ONEREVDIST * ((double)remainingCount / 562.25);  // in cm
          moveForward_custom(dist, true);
        }
        // robot veered to the right; obstacle on the right
        else if (dist_D2 < FRONT_1GRID_DIST + 2.5 && dist_D3 > FRONT_1GRID_END+1.0) {  // front-right < 7.5; front-left not blocked
          // 2. rotate left by >90
          rotateLeft_custom(90, 12);
          //rotateLeft(90);

          delay(80);

          // 3. move forward abit
          if (dist_D1 < FRONT_1GRID_DIST + 2.5) // if middle was also blocked, move forward abit more
            moveForward_custom(10.5, true);
          else  // just move about half a grid
            moveForward_custom(7.0, true);

          delay(80);

          // 4. rotate right
          rotateRight(90);

          delay(80);

          // 5. move remaining
          int remainingCount = targetCount - encL_curr;
          double dist = ONEREVDIST * ((double)remainingCount / 562.25);  // in cm
          moveForward_custom(dist, true);
        }
          
        // edge case: only middle blocked or all sensors blocked; not being handled
        // idea: save the current readings, wait for next command to come in which has to be a rotation,
        // then execute the recovery right after that rotation
        
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
  if ((/*dist_D1 > 3.0 &&*/ dist_D1 < FRONT_1GRID_DIST + 3.0) ||  // offset to account for robot's speed
    (/*dist_D2 > 3.0 &&*/ dist_D2 < FRONT_1GRID_DIST + 3.0) ||
    (/*dist_D3 > 3.0 &&*/ dist_D3 < FRONT_1GRID_DIST + 3.0)) 
    {
      md.setBrakes(BRAKE_L, BRAKE_R);
      emergencyBrakes = true;
      return; // stop checking
    }
}

// check that robot is in the center of the grids
void checkCentralise_Front() {

  double dist_D1, dist_D2, dist_D3;
  
// check for ideal distance from obstacle
  // check if too near to the wall right in front
  dist_D1 = front_D1.getDistance();
  dist_D2 = front_D2.getDistance();
  dist_D3 = front_D3.getDistance();
  if ((dist_D1 < FRONT_1GRID_DIST - ALIGN_FRONT_THRESHOLD)
    //|| (dist_D1 > (FRONT_1GRID_DIST + ALIGN_FRONT_THRESHOLD) && dist_D1 < FRONT_1GRID_END)
  ) { // front-mid sensor
    alignBack_Front(SharpIR::D1, FRONT_1GRID_DIST);
    didCentralise = true;
  }
  else if ((dist_D2 < FRONT_1GRID_DIST - ALIGN_FRONT_THRESHOLD)
    //|| (dist_D2 > (FRONT_1GRID_DIST + ALIGN_FRONT_THRESHOLD) && dist_D2 < FRONT_1GRID_END)
  ) {  // front-right sensor
    alignBack_Front(SharpIR::D2, FRONT_1GRID_DIST);
    didCentralise = true;
  }
  else if ((dist_D3 < FRONT_1GRID_DIST - ALIGN_FRONT_THRESHOLD)
    //|| (dist_D3 > (FRONT_1GRID_DIST + ALIGN_FRONT_THRESHOLD) && dist_D3 < FRONT_1GRID_END)
  ) {  // front-left sensor
    alignBack_Front(SharpIR::D3, FRONT_1GRID_DIST);
    didCentralise = true;
  }

  // check if too far from wall right in front
  dist_D1 = front_D1.getDistance();
  dist_D2 = front_D2.getDistance();
  dist_D3 = front_D3.getDistance();
  
  if (dist_D1 > (FRONT_1GRID_DIST + ALIGN_FRONT_THRESHOLD) && dist_D1 < FRONT_1GRID_END) { // front-mid sensor
    alignForward_Front(SharpIR::D1, FRONT_1GRID_DIST);
    didCentralise = true;
  }
  else if (dist_D2 > (FRONT_1GRID_DIST + ALIGN_FRONT_THRESHOLD) && dist_D2 < FRONT_1GRID_END) {  // front-right sensor
    alignForward_Front(SharpIR::D2, FRONT_1GRID_DIST);
    didCentralise = true;
  }
  else if (dist_D3 > (FRONT_1GRID_DIST + ALIGN_FRONT_THRESHOLD) && dist_D3 < FRONT_1GRID_END) {  // front-left sensor
    alignForward_Front(SharpIR::D3, FRONT_1GRID_DIST);
    didCentralise = true;
  }
}

void checkCentralise_Sides() {
  
  double dist_S1, dist_S2, dist_LR;

//check for ideal distance in middle of 3x3 grid
  //bool alignedWithLeft = false;
  bool checkedLeft = false;
  // check if too near or too far to the wall on left side
  dist_S1 = left_S1.getDistance();
  dist_S2 = left_S2.getDistance();

  if ((dist_S1 < LEFT_1GRID_END) || (dist_S2 < LEFT_1GRID_END)) {
    checkedLeft = true;
  }
  
  if ((dist_S1 < LEFT_1GRID_DIST - SIDES_DIST_THRESHOLD)
  ) { // S1 sensor < 3.9 //3.75
    rotateLeft(90);
    delay(100);
    alignBack_Front(SharpIR::D2, LEFT_1GRID_DIST);  // align with right sensor
    delay(70);
    // do alignment before turning back
    checkForTilted(); // in case robot isn't able to align after rotating back
    delay(50);
    rotateRight(90);
    delay(100);

    //alignedWithLeft = true;
    didCentralise = true;
  }
  else if ((dist_S2 < LEFT_1GRID_DIST - SIDES_DIST_THRESHOLD)
  ) {  // S2 sensor < 3.9 //3.75
    rotateLeft(90);
    delay(100);
    alignBack_Front(SharpIR::D1, LEFT_1GRID_DIST);  // align with middle sensor
    delay(70);
    // do alignment before turning back
    checkForTilted(); // in case robot isn't able to align after rotating back
    delay(50);
    rotateRight(90);
    delay(100);
    
    //alignedWithLeft = true;
    didCentralise = true;
  }
  // check if too far from wall on left side
  else if (dist_S1 > LEFT_1GRID_DIST + SIDES_DIST_THRESHOLD+0.5 && dist_S1 < LEFT_1GRID_END) { // S1 sensor > 6.4 //6.25
    rotateLeft(90);
    delay(100);
    alignForward_Front(SharpIR::D2, LEFT_1GRID_DIST);  // align with right sensor
    delay(70);
    // do alignment before turning back
    checkForTilted(); // in case robot isn't able to align after rotating back
    delay(50);
    rotateRight(90);
    delay(100);

    //alignedWithLeft = true;
    didCentralise = true;
  }
  else if (dist_S2 > LEFT_1GRID_DIST + SIDES_DIST_THRESHOLD+0.5 && dist_S2 < LEFT_1GRID_END) {  // S2 sensor > 6.4 //6.25
    rotateLeft(90);
    delay(100);
    alignForward_Front(SharpIR::D1, LEFT_1GRID_DIST);  // align with middle sensor
    delay(70);
    // do alignment before turning back
    checkForTilted(); // in case robot isn't able to align after rotating back
    delay(50);
    rotateRight(90);
    delay(100);

    //alignedWithLeft = true;
    didCentralise = true;
  }
  
  // check if too near or too far to the wall on right side - only if left-side calibration not used
  //if (!alignedWithLeft) {
  if (!checkedLeft) {
    dist_LR = right_long.getDistance();
    if ((dist_LR < RIGHT_1GRID_DIST - LR_SIDES_DIST_THRESHOLD)
    ) { // right LR sensor < 3.5
      rotateRight(90);
      delay(100);
      alignBack_Front(SharpIR::D3, RIGHT_1GRID_DIST);  // align with left sensor
      delay(70);
      // do alignment before turning back
      checkForTilted(); // in case robot isn't able to align after rotating back
      delay(50);
      rotateLeft(90);
      delay(100);
      
      didCentralise = true;
    }
    // check if too far from wall on right side
    else if (dist_LR > RIGHT_1GRID_DIST + LR_SIDES_DIST_THRESHOLD+0.5 && dist_LR < RIGHT_1GRID_END) { // right LR sensor > 6.4
      rotateRight(90);
      delay(100);
      alignForward_Front(SharpIR::D3, RIGHT_1GRID_DIST);  // align with left sensor
      delay(70);
      // do alignment before turning back
      checkForTilted(); // in case robot isn't able to align after rotating back
      delay(50);
      rotateLeft(90);
      delay(100);
      
      didCentralise = true;
    }
  }
}

// check if robot is tilted, i.e. not straight on the grids
void checkForTilted() {
// if front sensors can be used, use them instead of side
  double dist_D1 = front_D1.getDistance();
  double dist_D2 = front_D2.getDistance();
  double dist_D3 = front_D3.getDistance();

//  if ((dist_D1 >= 1.0 && dist_D1 <= 8.5 && dist_D2 >= 1.0 && dist_D2 <= 8.5) ||
//    (dist_D1 >= 13.0 && dist_D1 <= 18.0 && dist_D2 >= 13.0 && dist_D2 <= 18.0) ||
//    //|| (dist_D1 >= 23.0 && dist_D1 <= 28.0 && dist_D2 >= 23.0 && dist_D2 <= 28.0)
//    (dist_D1 >= 1.0 && dist_D1 <= 8.5 && dist_D3 >= 1.0 && dist_D3 <= 8.5) ||
//    (dist_D1 >= 13.0 && dist_D1 <= 18.0 && dist_D3 >= 13.0 && dist_D3 <= 18.0)
//    //|| (dist_D1 >= 23.0 && dist_D1 <= 28.0 && dist_D3 >= 23.0 && dist_D3 <= 28.0
//  ) {
//    canCheckCentralise_Sides = true;
//  }

  bool alignedWithFront = false;

  double difference1 = abs(dist_D1 - dist_D2);
  double difference2 = abs(dist_D1 - dist_D3);
  // front right
  if ((dist_D1 >= 1.0 && dist_D1 <= 8.5 && dist_D2 >= 1.0 && dist_D2 <= 8.5 && difference1 > 0.35) ||
    (dist_D1 >= 12.0 && dist_D1 <= 18.0 && dist_D2 >= 12.0 && dist_D2 <= 18.0 && difference1 > 0.55)
    //|| (dist_D1 >= 23.0 && dist_D1 <= 28.0 && dist_D2 >= 23.0 && dist_D2 <= 28.0 && difference1 > 0.8)
  )  // only try to align when within certain range from obstacle in front
  { // TODO: distance difference thresholds
    alignToFrontWall(false);  // align with right
    alignedWithFront = true;
  }
  else if (dist_D1 >= 1.0 && dist_D1 <= 8.5 && dist_D2 >= 12.0 && dist_D2 <= 18.0 && difference1 > 6.3) {
    alignToFrontWall(false, 10.0);  // align with right
    alignedWithFront = true;
  }
  else if (dist_D1 >= 12.0 && dist_D1 <= 18.0 && dist_D2 >= 1.0 && dist_D2 <= 8.5 && difference1 > 6.3) {
    alignToFrontWall(false, -10.0);  // align with right
    alignedWithFront = true;
  }
  // front left
  else if ((dist_D1 >= 1.0 && dist_D1 <= 8.5 && dist_D3 >= 1.0 && dist_D3 <= 8.5 && difference2 > 0.25) ||
    (dist_D1 >= 12.0 && dist_D1 <= 18.0 && dist_D3 >= 12.0 && dist_D3 <= 18.0 && difference2 > 0.55)
    //|| (dist_D1 >= 23.0 && dist_D1 <= 28.0 && dist_D3 >= 23.0 && dist_D3 <= 28.0 && difference2 > 0.8)
  )  // only try to align when within certain range from obstacle in front
  { // TODO: distance difference thresholds
      alignToFrontWall(true); // align with left
      alignedWithFront = true;
  }
  else if (dist_D1 >= 1.0 && dist_D1 <= 8.5 && dist_D3 >= 12.0 && dist_D3 <= 18.0 && difference2 > 6.3) {
    alignToFrontWall(true, -10.0);  // align with right
    alignedWithFront = true;
  }
  else if (dist_D1 >= 12.0 && dist_D1 <= 18.0 && dist_D3 >= 1.0 && dist_D3 <= 8.5 && difference2 > 6.3) {
    alignToFrontWall(true, 10.0);  // align with right
    alignedWithFront = true;
  }

// else, check if left side can be used for alignment
  if (!alignedWithFront) {
    double dist_S1 = left_S1.getDistance();
    double dist_S2 = left_S2.getDistance();
    double difference = dist_S2 - dist_S1;
    double abs_difference = abs(difference);

//    if ((dist_S1 >= 1.8 && dist_S1 <= 8.0 && dist_S2 >= 1.8 && dist_S2 <= 8.0) ||
//    (dist_S1 >= 12.0 && dist_S1 <= 18.0 && dist_S2 >= 12.0 && dist_S2 <= 18.0)
//    //|| (dist_S1 >= 23.0 && dist_S1 <= 28.0 && dist_S2 >= 23.0 && dist_S2 <= 28.0)
//    ) {
//      canCheckCentralise_Sides = true;
//    }
    
    if ((dist_S1 >= 1.8 && dist_S1 <= 8.0 && dist_S2 >= 1.8 && dist_S2 <= 8.0 && ((difference > 0.25 && difference < 5.5) || (difference < -0.1 && difference > -5.5))) ||  // by right, difference should be around >0.15
    (dist_S1 >= 12.0 && dist_S1 <= 18.0 && dist_S2 >= 12.0 && dist_S2 <= 18.0 && ((difference > 0.3 && difference < 5.5) || (difference < -0.1 && difference > -5.5)))  // difference originally listed as >0.2
    //|| (dist_S1 >= 23.0 && dist_S1 <= 28.0 && dist_S2 >= 23.0 && dist_S2 <= 28.0 && (difference > 0.3 || difference < 0))
    ) { // TODO: distance difference threshold
      alignToLeftWall();
    }
    else if ((dist_S1 >= 1.8 && dist_S1 <= 8.0 && dist_S2 >= 12.0 && dist_S2 <= 18.0) && (abs_difference > 6.15 && abs_difference < 15.5)) {
      alignToLeftWall(10.0);
    }
    else if ((dist_S1 >= 12.0 && dist_S1 <= 18.0 && dist_S2 >= 1.8 && dist_S2 <= 8.0) && (abs_difference > 6.15 && abs_difference < 15.5)) {
      alignToLeftWall(-10.0);
    }
  }
}

// Robot self-calibration: wall alignment
void alignToLeftWall(double distDiff = 0.0) {
  
  // use side sensors for alignment
  double dist_S1, dist_S2, difference;

  // 0: have not started moving; 1: diff > 0; 2: diff < 0
  char currState = 0, newState = 0;

  unsigned long startTime = millis();
  
  while ((millis() - startTime) < 400) {
    dist_S1 = left_S1.getDistance();
    dist_S2 = left_S2.getDistance();

    difference = dist_S2 - dist_S1 - distDiff;

    // abs: is a macro, so should be efficient
    if (0.075 <=difference && difference <= SIDE_THRESHOLD) { // very small gap, so stop moving
//    if (abs(difference) <= THRESHOLD) { // for S1 standing: very small gap, so stop moving
      //delay(1); // 1ms
      md.setBrakes(BRAKE_L, BRAKE_R);
      break;
    }
    else {
      newState = (difference < 0.075) ? STATE_DIFFGT0 : STATE_DIFFLT0;
//      newState = (difference > 0) ? STATE_DIFFGT0 : STATE_DIFFLT0;  // for S1 standing
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

void alignToFrontWall(bool useLeft, double distDiff = 0.0) {
  // use front-left (D3) and front-mid (D1) for alignment
  double dist_D1, dist_Dx, difference;

  // 0: have not started moving; 1: diff > 0; 2: diff < 0
  char currState = 0, newState = 0;
  
  unsigned long startTime = millis();
  
  while ((millis() - startTime) < 400) {
    dist_D1 = front_D1.getDistance();
    dist_Dx = useLeft ? front_D3.getDistance() : front_D2.getDistance();

    difference = useLeft ? dist_D1 - dist_Dx - distDiff : dist_Dx - dist_D1 - distDiff;

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
void alignBack_Front(SharpIR::sensorCode sensor, double targetDist) {
  
  // use one of the front sensors for alignment
  double dist_Dx = 0.0;
  
  unsigned long startTime = millis();

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
  } while (dist_Dx < targetDist && dist_Dx > targetDist - 5 && (millis() - startTime) < 400);
  md.setBrakes(BRAKE_L, BRAKE_R);
}

void alignForward_Front(SharpIR::sensorCode sensor, double targetDist) {
  
  // use one of the front sensors for alignment
  double dist_Dx = 0.0;
  
  unsigned long startTime = millis();

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
  } while (dist_Dx > targetDist && dist_Dx < targetDist + 5 && (millis() - startTime) < 400);
  md.setBrakes(BRAKE_L, BRAKE_R);
}
