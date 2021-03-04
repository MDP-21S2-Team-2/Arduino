#ifndef Movement_H
#define Movement_H

#define BRAKE_L 400
#define BRAKE_R 400

// comment away if doing FP
//#define EXPLORATION_MODE

//Declare Variables
extern bool emergencyBrakes;

// pre-calculated tEncodeVal for moveForward
const int tEncodeVal_lut[20] = {
  268, 568, 866, 1165, 1462, // 1-5 units
  1761, 2059, 2357, 2656, 2954, // 6-10 units
  3252, 3550, 3849, 4147, 4445, // 11-15 units
  4744, 5042, 5340, 5638, 5937  // 16-20 units
};

//Movement Variables
//double oneRevDis = 18.849556; // in cm

// movement
void moveForward(int moveUnits);
void moveBackward(int moveUnits);
// rotation
void rotateRight(int angle);
void rotateLeft(int angle);
// check for alignment
void initialGridCalibration();
void checkAlignmentAfterMove();
void checkAlignmentAfterRotate();

#endif
