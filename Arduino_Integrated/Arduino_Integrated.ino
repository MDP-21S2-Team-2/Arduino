#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>

#include "PID.h"
#include "SharpIR.h"

SharpIR front_D1(SharpIR::D1, A0);  // PS1, front right
SharpIR front_D2(SharpIR::D2, A1);  // PS2, front middle
SharpIR front_D3(SharpIR::D3, A2);  // PS3, front left
SharpIR left_S1(SharpIR::S1, A3); // PS4, side front
SharpIR left_S2(SharpIR::S2, A4); // PS5, side back
SharpIR right_long(SharpIR::LR, A5); // PS6, long range

String input;
DualVNH5019MotorShield md;
// pins for the motors' encoder channels
#define LeftMotorE1A 11
#define RightMotorE2A 3

#define BRAKE_L 400
#define BRAKE_R 400

// Counts for Encoder
volatile int encL_count = 0;
volatile int encR_count = 0;

// variables & ISR for calculating RPM
// Left
//volatile double L_rpm = 0.0;
volatile unsigned long L_prevTime = 0;
volatile unsigned long L_currTime = 0;
volatile unsigned long L_timeWidth = 0;
/*void calculateRpm_L() {
  L_currTime = micros();
  L_timeWidth = (L_currTime - L_prevTime);
  L_prevTime = L_currTime;
  double wheelRevolutionDuration = (L_timeWidth / 1000000.0) * 562.25;  // in seconds
  L_rpm = 60.0 / wheelRevolutionDuration;
}*/
// Right
//volatile double R_rpm = 0.0;
volatile unsigned long R_prevTime = 0;
volatile unsigned long R_currTime = 0;
volatile unsigned long R_timeWidth = 0;

// target speed for motors to reach
//const int targetPulseWidth = 821; //rpm=100: 1067;//rpm=130: 821;  // 60 / 130 / 562.25 * 1000000.0;
double targetRpm = 100.0;
//double targetDuration = 0.46154;  // for 1 rpm

double calculateRpm(int pulseWidth) {
  if (pulseWidth == 0) 
     return 0;
  else
    return 60000000.0 / (pulseWidth  * 562.25);  // time for 1 revolution in seconds
  //to get RPM, 60 / <above value>
}

const int alpha = 1;
const int alphaInv = 9;

void motorR_ISR() {
  encR_count++;
  R_currTime = micros();
  R_timeWidth = alpha * (R_currTime - R_prevTime) + alphaInv * R_timeWidth;
  R_timeWidth /= 10;
  R_prevTime = R_currTime;
}

void motorL_ISR() {
  L_currTime = micros();
  encL_count++;
  L_timeWidth = alpha * (L_currTime - L_prevTime) + alphaInv * L_timeWidth;
  L_timeWidth /= 10;
  //L_timeWidth = L_currTime - L_prevTime;
  L_prevTime = L_currTime;
}

// PID controller for each motor
// parameter list: P, I, D, Imax, Imin
// tuning for: 2yellowcap battery starting @ 6.3V while running, can hit 140-150+ RPM??

//PID leftPIDController(2.1, 2.6, 5.5, 130.0, -130); // red // for 130rpm
//PID leftPIDController(1.3, 2.2, 5.1, 130.0, -130); // red // for 130rpm
// 6.2V
PID leftPIDController(1.1, 2.035, 0.15, 130.0, -130); // red // for 130rpm // initially: I = 2.015
//PID rightPIDController(0.8, 2.02, 5.1, 130.0, -130.0); // for 130rpm// blue
PID rightPIDController(1.3, 1.976, 1.0, 130.0, -130.0); // for 130rpm// blue

//PID rightPIDController(2.1, 2.6, 0.1, 130.0, -130.0); // for 130rpm// blue

// Distance Function
//double leftWheelDiameter = 6.0;   // in cm
//double rightWheelDiameter = 6.0;  // in cm
//Wheel to wheel distance = 18.5cm
//Circumference of whole robot = 58.11cm
// 90 degree = 14.5275cm 
double oneRevDis = 18.849556; // in cm

void moveForward(double tDistance)
{
  //Store current encoder value
  int fLval = encL_count;
  int fRval = encR_count;
  // Calculate target number of ticks to travel the distance,
  // reduce tEncodeVal by no. ticks needed for braking
  double L_tEncodeVal = fLval + tDistance/oneRevDis * 562.25; //- 35;  // 38
  double R_tEncodeVal = fRval + tDistance/oneRevDis * 562.25; //- 36;  // 33

  // reset prevTime to get more accurate timeWidth
  L_prevTime = micros();
  R_prevTime = micros();
  
  // check if either motor reached the target number of ticks
  while ((encL_count <= L_tEncodeVal) || (encR_count<= R_tEncodeVal))
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
}
void moveBackward(double tDistance)
{
  //Store current encoder value
  int fLval = encL_count;
  int fRval = encR_count;
  // Calculate target number of ticks to travel the distance,
  // reduce tEncodeVal by no. ticks needed for braking
  double L_tEncodeVal = fLval + tDistance/oneRevDis * 562.25; //- 35;  // 38
  double R_tEncodeVal = fRval + tDistance/oneRevDis * 562.25; //- 36;  // 33

  // check if either motor reached the target number of ticks
  while ((encL_count <= L_tEncodeVal) || (encR_count<= R_tEncodeVal))
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(-rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
}

void rotateLeft(double angle)
{
//  //Store current encoder value
//  int curLeftEnc = encL_count;
//  int curRightEnc = encR_count;
  double target_count = (encL_count+encR_count)/2;
  Serial.print("Target count: ");
  Serial.println(target_count);
  //4.8147 exact multiplier
  // Calculate target number of ticks to travel the distance,
  // reduce tEncodeVal by no. ticks needed for braking
  // Every degree takes (1/360 *58.11 /18.84956 * 562.25) = 4.8147
  // check if either motor reached the target number of ticksif (angle <=90)
  if (angle <= 90)
    target_count += angle * 4.71;
  else if ( 90< angle <= 180)
    target_count += angle * 4.70;

  // reset prevTime to get more accurate timeWidth
  L_prevTime = micros();
  R_prevTime = micros();
  
  while ((encL_count + encR_count)/2 <= target_count)
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
}

void rotateRight(double angle)
{
  //Store current encoder value
  int curLeftEnc = encL_count;
  int curRightEnc = encR_count;
  double target_count = 0;
  //4.8147 exact multiplier
  // Calculate target number of ticks to travel the distance,
  // reduce tEncodeVal by no. ticks needed for braking
  // Every degree takes (1/360 *58.11 /18.84956 * 562.25) = 4.8147
  // check if either motor reached the target number of ticksif (angle <=90)
  if (angle <= 90)
    target_count = angle * 4.61;
  else if ( 90< angle <= 180)
    target_count = angle * 4.70;
    
  // reset prevTime to get more accurate timeWidth
  L_prevTime = micros();
  R_prevTime = micros();

  while ((encL_count <= target_count) || (encR_count <= target_count))
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(-rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
}

unsigned long startTime = 0;

void setup() {
  // put your setup code here, to run once:
  //init encoder pins
  //(default is input so technically not needed)
  //pinMode(LeftMotorE1A, INPUT);
  //pinMode(RightMotorE2A, INPUT);
  
  // enable interrupts for calculating motor rpm
  enableInterrupt(LeftMotorE1A, motorL_ISR, RISING);
  enableInterrupt(RightMotorE2A, motorR_ISR, RISING);
  
  // init Serial
  Serial.begin(115200);
  // init motor drivers
  md.init();

// left
  //md.setM1Speed(-400);
// right
  //md.setM2Speed(400);

  // set starting times
  L_prevTime = micros();
  R_prevTime = L_prevTime;

  startTime = micros();
}


const int loopTime = 20;  // example in ms

void loop() {
//  if (micros() - startTime > 1500000) {}
//  else if (encL_count>=620) //310 = 10cm, // 562 = 1 rev
//  {//micros() - startTime >= 1000000) {
//     //stop
//    md.setBrakes(400, 400);
//    Serial.print(encL_count);
//    Serial.print(",");
//    Serial.println(encR_count);
//  }
//  else {
//    if (PID::checkPIDCompute()) {
//    // PID
//      md.setM1Speed(-leftPIDController.computePID(L_timeWidth, targetPulseWidth));
//      md.setM2Speed(rightPIDController.computePID(R_timeWidth, targetPulseWidth));
//    }
//  }

//for (int i = 0; i <4 ; i++){
//  delay(2000);
//    moveForward(10);
//    leftPIDController.resetPID();
//    rightPIDController.resetPID();
//  }
  //testInLoop_motorsPID();
 testInLoop_readingIR();

 //Serial.println(right_long.getDistance(true));
}

void testInLoop_readingIR() {
    //delay(100);
  
  //Serial.println(front_D1.getDistance(true));
  //Serial.println(front_D2.getDistance(true));
  //Serial.println(front_D3.getDistance(true));

//  Serial.print("Front, right: ");
//  Serial.println(front_D1.getDistance(true));
//  Serial.print("Front, middle: ");
//  Serial.println(front_D2.getDistance(true));
  Serial.println(left_S1.getDistance(true));
//  Serial.print(left_S2.getDistance(true));
//  Serial.println(right_long.getDistance(true));
  delay(20);  // frequency = ?
//  
}

void testInLoop_motorsPID() {
  if (PID::checkPIDCompute()) {
    // send RPM of left motor
    //R_rpm = calculateRpm_R_v2();
    //Serial.println(60 / R_rpm);
  
    //Serial.println(R_timeWidth);
    //Serial.println(R_timeWidth);
  
    double R_rpm = calculateRpm(R_timeWidth);
    double L_rpm = calculateRpm(L_timeWidth);
    
    Serial.print(R_rpm);
    Serial.print(",");
    Serial.println(L_rpm);
  
    //md.setM1Speed(-350);
    //md.setM2Speed(350);
    // update motor speed with PID controller
    md.setM1Speed(-leftPIDController.computePID(L_rpm, targetRpm));
    md.setM2Speed(rightPIDController.computePID(R_rpm, targetRpm));
  }
}
