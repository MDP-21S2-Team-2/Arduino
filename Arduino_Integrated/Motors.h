#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <DualVNH5019MotorShield.h>
#include "PID.h"

// pins for the motors' encoder channels
#define LeftMotorA 11 // E2A
#define RightMotorA 3 // E1A

//#define targetRpm TARGETRPM_120
#define targetRpm TARGETRPM_125

// Setting Target RPM 110 RPM
#define TARGETRPM_110 110
// Setting Target RPM 120 RPM
#define TARGETRPM_120 120
// Setting Target RPM 125 RPM
#define TARGETRPM_125 125

// offset ticks to call setBrakes early
#define MOVE_OFFTICKS 24  // 120 RPM
//#define MOVE_OFFTICKS 20  // 100 RPM
#define ONEREVDIST 18.849556


// Motor shield
extern DualVNH5019MotorShield md;

//Counts for encoder
extern volatile unsigned char encL_count;
extern volatile unsigned char encR_count;
extern volatile unsigned char encL_curr_count;
extern volatile unsigned char encR_curr_count;
extern volatile unsigned char encL_overshootCount;
extern volatile unsigned char encR_overshootCount;

// Left
extern volatile unsigned long L_prevTime;
extern volatile unsigned long L_currTime;
extern volatile unsigned long L_timeWidth;

// Right
extern volatile unsigned long R_prevTime;
extern volatile unsigned long R_currTime;
extern volatile unsigned long R_timeWidth;

// PID controller for each motor
extern PID leftPIDController;
extern PID rightPIDController;

const float alpha = 0.3;
const float alphaInv = 0.7;

void resetEnc();
void resetPIDControllers();
double calculateRpm(int pulseWidth);

// ISR routines for motor encoder
void motorR_ISR();
void motorL_ISR();

// compute no. units moved forward
int computeUnitsMoved();
void sendUnitsMoved(int units);

#endif
