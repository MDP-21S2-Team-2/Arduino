#ifndef Movement_H
#define Movement_H

#define BRAKE_L 400
#define BRAKE_R 400

// comment away if doing FP
//#define EXPLORATION_MODE

//Declare Variables
extern bool emergencyBrakes;

// pre-calculated tEncodeVal for moveForward
//const int tEncodeVal_lut[20] = {
//  268, 568, 866, 1165, 1462, // 1-5 units
//  1761, 2059, 2357, 2656, 2954, // 6-10 units
//  3252, 3550, 3849, 4147, 4445, // 11-15 units
//  4744, 5042, 5340, 5638, 5937  // 16-20 units
//};

const int numOvershoot_lut[20] = {
  1, 2, 3, 4, 5, // 1-5 units
  6, 8, 9, 10, 11, // 6-10 units
  12, 13, 15, 16, 17, // 11-15 units
  18, 19, 20, 22, 23  // 16-20 units
};
const int remainderCount_lut[20] = {
  12, 56, 98, 141, 182,
  225, 11, 53, 96, 138,
  180, 222, 9, 51, 93,
  136, 178, 220, 6, 49
};

//Movement Variables
//double oneRevDis = 18.849556; // in cm

// movement
void moveForward(int moveUnits, bool emergencyEnabled);
void moveBackward(int moveUnits);
// rotation
void rotateRight(int angle);
void rotateLeft(int angle);
// check for alignment
void initialGridCalibration();
void checkAlignmentAfterMove();
void checkAlignmentAfterRotate();

#endif
