#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <DualVNH5019MotorShield.h>
#include "PID.h"

// pins for the motors' encoder channels
#define LeftMotorA 11 // E2A
#define RightMotorA 3 // E1A

// Setting Target RPM 125 RPM
#define targetRpm 125.0
// Setting Target RPM 130 RPM
//#define targetRpm 135.0

//#define MOVE_OFFTICKS 24
//#define MOVE_OFFTICKS 20


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

extern const float alpha;
extern const float alphaInv;

void resetEnc();
void resetPIDControllers();
double calculateRpm(int pulseWidth);

// ISR routines for motor encoder
void motorR_ISR();
void motorL_ISR();

#endif
