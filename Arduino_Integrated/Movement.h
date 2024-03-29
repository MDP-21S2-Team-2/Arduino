#ifndef Movement_H
#define Movement_H

#define BRAKE_L 400
#define BRAKE_R 400

#define EXPLORATION_MODE  // comment away if doing FP
//#define IMAGEREC_MODE // comment away if not doing ImgRec

//Declare Variables
extern bool emergencyBrakes;

// pre-calculated tEncodeVal for moveForward
//const int tEncodeVal_lut[20] = {
//  268, 568, 866, 1165, 1462, // 1-5 units
//  1761, 2059, 2357, 2656, 2954, // 6-10 units
//  3252, 3550, 3849, 4147, 4445, // 11-15 units
//  4744, 5042, 5340, 5638, 5937  // 16-20 units
//};

#if targetRpm == TARGETRPM_110
  // for 110 RPM
  const int numOvershoot_lut[20] = {
    1, 2, 3, 4, 5, // 1-5 units
    6, 8, 9, 10, 11, // 6-10 units
    12, 13, 15, 16, 17, // 11-15 units
    18, 19, 20, 22, 23  // 16-20 units
  };
  const int remainderCount_lut[20] = {
    15, 59, 100, 143, 185,
    228, 15, 57, 100, 142,
    184, 226, 13, 55, 97,
    140, 182, 224, 10, 53
  };
  
  // for checking LR - 4 ticks less than actual ticks required to cover the distance
  const int total_lut[20] = {
  267, 567, 864,  // 1-3 units
  1165, 1462, // 4-5 units
  1761, 2059, 2357, 2656, 2954, // 6-10 units
  3252, 3550, 3849, 4147, 4445, // 11-15 units
  4744, 5042, 5340, 5638, 5937  // 16-20 units
  };
#elif targetRpm == TARGETRPM_120
  // for 120 RPM
  const int numOvershoot_lut[20] = {
    1, 2, 3, 4, 5, // 1-5 units
    6, 8, 9, 10, 11, // 6-10 units
    12, 13, 15, 16, 17, // 11-15 units
    18, 19, 20, 22, 23  // 16-20 units
  };
  const int remainderCount_lut[20] = {
    19, 62, 102, 147, 189, //move 3 units changed from 104 to 102
    231, 17, 59, 102, 144,
    186, 228, 15, 57, 99,
    142, 184, 226, 12, 55
  };

  // for checking LR - 4 ticks less than actual ticks required to cover the distance
  const int total_lut[20] = {
  271, 570, 866,  // 1-3 units
  1165, 1462, // 1-5 units
  1761, 2059, 2357, 2656, 2954, // 6-10 units
  3252, 3550, 3849, 4147, 4445, // 11-15 units
  4744, 5042, 5340, 5638, 5937  // 16-20 units
  };
#elif targetRpm == TARGETRPM_125
  // for 120 RPM
  const int numOvershoot_lut[20] = {
    1, 2, 3, 4, 5, // 1-5 units
    6, 8, 9, 10, 11, // 6-10 units
    12, 13, 15, 16, 17, // 11-15 units
    18, 19, 20, 22, 23  // 16-20 units
  };
  const int remainderCount_lut[20] = {
    18, 61, 101, 146, 189, //move 3 units changed from 104 to 102
    231, 17, 59, 102, 144,
    186, 228, 15, 57, 99,
    142, 184, 226, 12, 55
  };

  // for checking LR - 4 ticks less than actual ticks required to cover the distance
  const int total_lut[20] = {
  270, 569, 865,  // 1-3 units
  1164, 1462, // 1-5 units
  1761, 2059, 2357, 2656, 2954, // 6-10 units
  3252, 3550, 3849, 4147, 4445, // 11-15 units
  4744, 5042, 5340, 5638, 5937  // 16-20 units
  };
#endif

// movement
bool moveForward(int moveUnits, bool emergencyEnabled, bool crashRecovery = true, unsigned int additionalTicks = 0);
void moveBackward(int moveUnits);
// rotation
void rotateRight(int angle);
void rotateLeft(int angle);
// CUSTOM VALUES FOR MOVEMENT
void moveForward_custom(double distance, bool emergencyEnabled);
void moveBackward_custom(double distance);
void rotateRight_custom(int angle, int tickOffset);
void rotateLeft_custom(int angle, int tickOffset);
// new movement commands
bool moveForward_W(int moveUnits, int *currStep); // for rushing along wall - Exp
// check for alignment
void initialGridCalibration();
void checkAlignmentAfterMove();
void checkAlignmentAfterRotate();
void checkAlignmentAfterCommand_FP();

#endif
