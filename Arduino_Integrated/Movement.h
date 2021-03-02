#ifndef Movement_H
#define Movement_H
#include <Arduino.h>
#include <DualVNH5019MotorShield.h>
#include "PID.h"
#define BRAKE_L 400
#define BRAKE_R 400

// comment away if doing FP
#define EXPLORATION_MODE

//Declare Variables
extern bool emergencyBrakes;
//Counts for encoder
extern volatile int encL_count;
extern volatile int encR_count;
extern volatile int encL_curr_count;
extern volatile int encR_curr_count;
extern PID leftPIDController;
extern PID rightPIDController;
extern DualVNH5019MotorShield md;

// pre-calculated tEncodeVal for moveForward
const int tEncodeVal_lut[20] = {
  269, 573, 871, 1169, 1467, // 1-5 units
  1766, 2064, 2362, 2661, 2959, // 6-10 units
  3257, 3555, 3854, 4152, 4450, // 11-15 units
  4749, 5047, 5345, 5643, 5942  // 16-20 units
};

//Movement Variables
//double oneRevDis = 18.849556; // in cm

// Left
extern volatile unsigned long L_prevTime;
extern volatile unsigned long L_currTime;
extern volatile unsigned long L_timeWidth;

// Right
extern volatile unsigned long R_prevTime;
extern volatile unsigned long R_currTime;
extern volatile unsigned long R_timeWidth;


void resetEnc();
double calculateRpm(int pulseWidth);
void moveForward(int moveUnits);
void moveBackward(int moveUnits);
void rotateRight2(double angle);
void rotateRight(int angle);
void rotateLeft(int angle);
void rotateLeft2(double angle);

#endif
