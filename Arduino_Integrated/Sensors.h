#ifndef Sensors_H
#define Sensors_H

#include "SharpIR.h"

extern SharpIR front_D1;  // PS1, front middle
extern SharpIR front_D2;  // PS2, front right
extern SharpIR front_D3;  // PS3, front left
extern SharpIR left_S1; // PS5, side front
extern SharpIR left_S2; // PS4, side back
extern SharpIR right_long; // PS6, long range

void sendIRSensorsReadings();

#endif
