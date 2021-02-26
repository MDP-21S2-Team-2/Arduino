#include "Sensors.h"


SharpIR front_D1(SharpIR::D1, A0);  // PS1, front middle
SharpIR front_D2(SharpIR::D2, A1);  // PS2, front right
SharpIR front_D3(SharpIR::D3, A2);  // PS3, front left
SharpIR left_S1(SharpIR::S1, A4); // PS5, side front
SharpIR left_S2(SharpIR::S2, A3); // PS4, side back
SharpIR right_long(SharpIR::LR, A5); // PS6, long range
