#ifndef Movement_H
#define Movement_H

#define BRAKE_L 400
#define BRAKE_R 400

// comment away if doing FP
#define EXPLORATION_MODE

//Declare Variables
extern bool emergencyBrakes;

// pre-calculated tEncodeVal for moveForward
//const int tEncodeVal_lut[20] = {
//  268, 568, 866, 1165, 1462, // 1-5 units
//  1761, 2059, 2357, 2656, 2954, // 6-10 units
//  3252, 3550, 3849, 4147, 4445, // 11-15 units
//  4744, 5042, 5340, 5638, 5937  // 16-20 units
//};

// for 110 RPM
//const int numOvershoot_lut[20] = {
//  1, 2, 3, 4, 5, // 1-5 units
//  6, 8, 9, 10, 11, // 6-10 units
//  12, 13, 15, 16, 17, // 11-15 units
//  18, 19, 20, 22, 23  // 16-20 units
//};
//const int remainderCount_lut[20] = {
//  15, 59, 100, 143, 185,
//  228, 15, 57, 100, 142,
//  184, 226, 13, 55, 97,
//  140, 182, 224, 10, 53
//};

// for 120 RPM
const int numOvershoot_lut[20] = {
  1, 2, 3, 4, 5, // 1-5 units
  6, 8, 9, 10, 11, // 6-10 units
  12, 13, 15, 16, 17, // 11-15 units
  18, 19, 20, 22, 23  // 16-20 units
};
const int remainderCount_lut[20] = {
  14, 58, 99, 142, 183,
  226, 12, 54, 97, 139,
  181, 223, 10, 52, 94,
  137, 179, 221, 7, 50
};

// movement
void moveForward(int moveUnits, bool emergencyEnabled);
void moveBackward(int moveUnits);
// rotation
void rotateRight(int angle);
void rotateLeft(int angle);
// CUSTOM VALUES FOR MOVEMENT
void moveForward_custom(double distance, bool emergencyEnabled);
void rotateRight_custom(int angle, int tickOffset);
void rotateLeft_custom(int angle, int tickOffset);
// new movement comamnds
void moveForward();
// check for alignment
void initialGridCalibration();
void checkAlignmentAfterMove();
void checkAlignmentAfterRotate();
void checkAlignmentAfterCommand_FP();

#endif
