#include "Sensors.h"

SharpIR front_D1(SharpIR::D1, A0);  // PS1, front middle
SharpIR front_D2(SharpIR::D2, A1);  // PS2, front right
SharpIR front_D3(SharpIR::D3, A2);  // PS3, front left
SharpIR left_S1(SharpIR::S1, A4); // PS5, side front
SharpIR left_S2(SharpIR::S2, A3); // PS4, side back
SharpIR right_long(SharpIR::LR, A5); // PS6, long range

void sendIRSensorsReadings() {

  double dist_D1 = front_D1.getDistance();
  double dist_D2 = front_D2.getDistance();
  double dist_D3 = front_D3.getDistance();
  double dist_S1 = left_S1.getDistance();
  double dist_S2 = left_S2.getDistance();
  double dist_LR = right_long.getDistance();

  // 296~308 micros
  byte* ptr_dist_D1 = (byte*) &dist_D1;
  byte* ptr_dist_D2 = (byte*) &dist_D2;
  byte* ptr_dist_D3 = (byte*) &dist_D3;
  byte* ptr_dist_S1 = (byte*) &dist_S1;
  byte* ptr_dist_S2 = (byte*) &dist_S2;
  byte* ptr_dist_LR = (byte*) &dist_LR;

  Serial.write("IR,");
  Serial.write(ptr_dist_S2, 4);
  Serial.write(",");
  Serial.write(ptr_dist_S1, 4);
  Serial.write(",");
  Serial.write(ptr_dist_D3, 4);
  Serial.write(",");
  Serial.write(ptr_dist_D1, 4);
  Serial.write(",");
  Serial.write(ptr_dist_D2, 4);
  Serial.write(",");
  Serial.write(ptr_dist_LR, 4);
  Serial.write('\n');

//  Serial.write("IR,");
//  Serial.print(dist_S2);
//  Serial.write(",");
//  Serial.print(dist_S1);
//  Serial.write(",");
//  Serial.print(dist_D3);
//  Serial.write(",");
//  Serial.print(dist_D1);
//  Serial.write(",");
//  Serial.print(dist_D2);
//  Serial.write(",");
//  Serial.println(dist_LR);
}

void sendRightSensorReadings() {
  
  double dist_LR = right_long.getDistance();
  byte* ptr_dist_LR = (byte*) &dist_LR;
  
  Serial.write("LR,");
  Serial.write(ptr_dist_LR, 4);
  Serial.write('\n');
  
//  Serial.println(dist_LR);
}
