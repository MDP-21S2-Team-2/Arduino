#ifndef Movement_H
#define Movement_H

#define BRAKE_L 400
#define BRAKE_R 400

// comment away if doing FP
#define EXPLORATION_MODE

//Declare Variables
extern bool emergencyBrakes;

// pre-calculated tEncodeVal for moveForward
const int tEncodeVal_lut[20] = {
  269, 569, 871, 1169, 1467, // 1-5 units
  1766, 2064, 2362, 2661, 2959, // 6-10 units
  3257, 3555, 3854, 4152, 4450, // 11-15 units
  4749, 5047, 5345, 5643, 5942  // 16-20 units
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
