#include <DualVNH5019MotorShield.h>

#include <EnableInterrupt.h>

#include "PID.h"
#include "SharpIR.h"

SharpIR front_D1(SharpIR::D1, A0);  // PS1, front middle
SharpIR front_D2(SharpIR::D2, A1);  // PS2, front right
SharpIR front_D3(SharpIR::D3, A2);  // PS3, front left
//S1 range until
SharpIR left_S1(SharpIR::S1, A4); // PS5, side front
// S2 range until 40
SharpIR left_S2(SharpIR::S2, A3); // PS4, side back
SharpIR right_long(SharpIR::LR, A5); // PS6, long range

DualVNH5019MotorShield md;
// pins for the motors' encoder channels
#define LeftMotorE1A 11
#define RightMotorE2A 3

#define BRAKE_L 400
#define BRAKE_R 400

// Counts for Encoder
volatile int encL_count = 0;
volatile int encR_count = 0;

// Serial communication
String input;

// variables & ISR for calculating RPM
// Left
//volatile double L_rpm = 0.0;
volatile unsigned long L_prevTime = 0;
volatile unsigned long L_currTime = 0;
volatile unsigned long L_timeWidth = 0;

// Right
//volatile double R_rpm = 0.0;
volatile unsigned long R_prevTime = 0;
volatile unsigned long R_currTime = 0;
volatile unsigned long R_timeWidth = 0;

// target speed for motors to reach
//const int targetPulseWidth = 821; //rpm=100: 1067;//rpm=130: 821;  // 60 / 130 / 562.25 * 1000000.0;
//double targetRpm = 125.0; // 100.0;
double targetRpm = 100.0;
double alignmentTargetRpm = 50.0;
//double targetDuration = 0.46154;  // for 1 rpm
double target_count = 0;
int leftSign = 1;
int rightSign = 1;

double calculateRpm(int pulseWidth) {
  if (pulseWidth == 0)
    return 0;
  else
    return 60000000.0 / (pulseWidth  * 562.25);  // time for 1 revolution in seconds
  //to get RPM, 60 / <above value>
}

const int alpha = 1;
const int alphaInv = 9;

void resetEnc() {
  encL_count = 0;
  encR_count = 0;
}

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
//PID leftPIDController(1.1, 2.035, 0.15, 130.0, -130); // red // for 130rpm // initially: I = 2.015
//PID rightPIDController(0.8, 2.02, 5.1, 130.0, -130.0); // for 130rpm// blue
//PID rightPIDController(1.3, 1.976, 1.0, 130.0, -130.0); // for 130rpm// blue
//PID rightPIDController(2.1, 2.6, 0.1, 130.0, -130.0); // for 130rpm// blue

//9 feb 6.35V 1y1w
// can move straight
//PID leftPIDController(0.95, 2.13, 5.6, 130.0, -130); // red // for 130rpm // initially: I = 2.015
//PID rightPIDController(0.92, 2.04, 5.5, 130.0, -130.0);

// 10 Feb 6.22V 1y1w
//PID leftPIDController(0.961, 2.1, 5.6, 130.0, -130); // red // for 130rpm // initially: I = 2.015
//PID rightPIDController(0.92, 2.04, 5.5, 130.0, -130.0);

// 10 Feb 6.42V 2y target speed:100
//PID leftPIDController(0.91, 2.02, 5.6, 130.0, -130); // red // for 130rpm // initially: I = 2.015
//PID rightPIDController(0.82, 1.96, 5.7, 130.0, -130.0);

// 10 Feb target speed:50 6.42V 2y
//PID leftPIDController(0.6, 2.68, 4.4, 50.0, -50); // red // for 130rpm // initially: I = 2.015
//PID rightPIDController(0.58, 2.65, 5.1, 50.0, -50.0);

// 11 Feb target speed:100 6.34V 2y, 6.31V after use
//PID leftPIDController(0.961, 2.055, 5.6, 130.0, -130); // red // for 130rpm // initially: I = 2.015
//PID rightPIDController(0.91, 1.985, 5.5, 130.0, -130.0);

// 15 Feb target speed:100 6.32V 2y, 6.26V after use
//PID leftPIDController(0.961, 2.071, 6.5, 130.0, -130); // red
//PID rightPIDController(0.91, 2.01, 5.5, 130.0, -130.0);

// 16 Feb target speed:120 6.33V 2y, 6.V after use
//PID leftPIDController(1.1, 2.617, 5.75, 130.0, -130); // red // initially 1.2, 2.63, 5.75
//PID rightPIDController(1.08, 2.52, 5.95, 130.0, -130.0);


// 17 Feb target speed:100 6.29V 2y, 6.25V after use
//PID leftPIDController(0.961, 2.078, 6, 130.0, -130); // red
//PID rightPIDController(0.91, 2.035, 6, 130.0, -130.0);

// 18 Feb target speed: 100 start: 6.29V 2y -- cmi battery level too low, right keeps starting faster sometimes asdfjakd
//PID leftPIDController(0.961, 2.078, 0.0, 130.0, -130); // red
//PID rightPIDController(0.91, 2.03, 0.0, 130.0, -130.0);

// 18 Feb target speed: 100 start: 6.4V 2y
//PID leftPIDController(0.961, 2.03, 6.0, 130.0, -130); // red
//PID rightPIDController(0.91, 1.97, 6.0, 130.0, -130.0);

// 18 Feb target speed:100 6.31V 1y1w
//PID leftPIDController(0.961, 2.0697, 5.6, 130.0, -130); // red // for 130rpm // initially: I = 2.015
//PID rightPIDController(0.91, 2.023, 5.5, 130.0, -130.0);

// 19 Feb target speed:100 6.39V 2y
//PID leftPIDController(0.961, 2.0225, 0.0, 130.0, -130); // red
//PID rightPIDController(0.7, 2.0215, 0.0, 130.0, -130.0); // right starts up faster

// 19 Feb target speed:100 6.39V 2y
PID leftPIDController(0.961, 2.045, 1.0, 130.0, -130); // red
PID rightPIDController(0.7, 2.017, 2.0, 130.0, -130.0); // right starts up faster

// Distance Function
//double leftWheelDiameter = 6.0;   // in cm
//double rightWheelDiameter = 6.0;  // in cm
//Wheel to wheel distance = 18.5cm
//Circumference of whole robot = 58.11cm
// 90 degree = 14.5275cm
double oneRevDis = 18.849556; // in cm

void checkForCalibration() {
  double dist_D1 = front_D1.getDistance();
  double dist_D2 = front_D2.getDistance();
  double dist_D3 = front_D3.getDistance();
  double dist_S1 = left_S1.getDistance();
  double dist_S2 = left_S2.getDistance();
  double dist_LR = right_long.getDistance();

  byte* ptr_dist_D1 = (byte*) &dist_D1;
  byte* ptr_dist_D2 = (byte*) &dist_D2;
  byte* ptr_dist_D3 = (byte*) &dist_D3;
  byte* ptr_dist_S1 = (byte*) &dist_S1;
  byte* ptr_dist_S2 = (byte*) &dist_S2;
  byte* ptr_dist_LR = (byte*) &dist_LR;
}

void moveForward(double tDistance)
{
  //Store current encoder value
  int fLval = encL_count;
  int fRval = encR_count;
  // Calculate target number of ticks to travel the distance,
  // reduce tEncodeVal by no. ticks needed for braking
  double L_tEncodeVal = fLval + tDistance / oneRevDis * 562.25; //- 35;  // 38
  double R_tEncodeVal = fRval + tDistance / oneRevDis * 562.25; //- 36;  // 33

  // set multiplier for motor speed direction
  leftSign = -1;
  rightSign = 1;

  // reset prevTime to get more accurate timeWidth
  L_prevTime = micros();
  R_prevTime = micros();

  // check if either motor reached the target number of ticks
  while ((encL_count <= L_tEncodeVal) || (encR_count <= R_tEncodeVal))
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
      //md.setSpeeds(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm), rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));

      // read IR sensors here
      checkForCalibration();
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);

  // reset PID
  leftPIDController.resetPID();
  rightPIDController.resetPID();
  // reset encoder ticks
  resetEnc();

  // TEMPORARY: SEND AFTER EVERY 1 UNIT
  //sendIRSensorsReadings();
}
void moveBackward(double tDistance)
{
  //Store current encoder value
  int fLval = encL_count;
  int fRval = encR_count;
  // Calculate target number of ticks to travel the distance,
  // reduce tEncodeVal by no. ticks needed for braking
  double L_tEncodeVal = fLval + tDistance / oneRevDis * 562.25; //- 35;  // 38
  double R_tEncodeVal = fRval + tDistance / oneRevDis * 562.25; //- 36;  // 33
  //target_count = (fLval+fRval)/2;
  //target_count += tDistance/oneRevDis * 562.25;

  // set multiplier for motor speed direction
  leftSign = 1;
  rightSign = -1;

  // reset prevTime to get more accurate timeWidth
  L_prevTime = micros();
  R_prevTime = micros();

  // check if either motor reached the target number of ticks
  while ((encL_count <= L_tEncodeVal) || (encR_count <= R_tEncodeVal))
  {
    if (PID::checkPIDCompute()) {
      //md.setM1Speed(leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      //md.setM2Speed(-rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
      md.setSpeeds(leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm), -rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
  
  // reset PID
  leftPIDController.resetPID();
  rightPIDController.resetPID();
  // reset encoder ticks
  resetEnc();
}

void rotateLeft2(double angle) {

  int fLval = encL_count;
  int fRval = encR_count;
  // Calculate target number of ticks to travel the distance,
  // reduce tEncodeVal by no. ticks needed for braking
  // was previously 4.45
  double L_tEncodeVal = fLval; //4.46; for brakes: //- 35;  // 38
  double R_tEncodeVal = fRval; //4.46; for brakes: //- 36;  // 33

  if (angle <= 50) {
    L_tEncodeVal += angle * 4.21;  // 4.27 works
    R_tEncodeVal += angle * 4.21;  // 4.27 works
  }
  else if (angle <= 91) {
  // 4.42 FKING SOLID 11 FEB 6.34V TUNING
    L_tEncodeVal += angle * 4.42;
    R_tEncodeVal += angle * 4.42;
  }
  else if (angle <= 180) {
    L_tEncodeVal += angle * 4.51;
    R_tEncodeVal += angle * 4.51;
  }

  // set multiplier for motor speed direction
  leftSign = 1;
  rightSign = 1;

  // reset prevTime to get more accurate timeWidth
  L_prevTime = micros();
  R_prevTime = micros();

  while ((encL_count <= L_tEncodeVal) || (encR_count <= R_tEncodeVal))
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
      //md.setSpeeds(leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm), rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
  
  // reset PID
  leftPIDController.resetPID();
  rightPIDController.resetPID();
  // reset encoder ticks
  resetEnc();
}

void rotateLeft(double angle)
{
  //  //Store current encoder value
  //  int curLeftEnc = encL_count;
  //  int curRightEnc = encR_count;
  target_count = (encL_count + encR_count) / 2;
  //4.8147 exact multiplier
  // Calculate target number of ticks to travel the distance,
  // reduce tEncodeVal by no. ticks needed for braking
  // Every degree takes (1/360 *58.11 /18.84956 * 562.25) = 4.8147
  // check if either motor reached the target number of ticksif (angle <=90)
  if (angle <= 90)
    target_count += angle * 4.41;
  else if ( 90 < angle <= 180)
    target_count += angle * 4.65;

  // set multiplier for motor speed direction
  leftSign = 1;
  rightSign = 1;

  // reset prevTime to get more accurate timeWidth
  L_prevTime = micros();
  R_prevTime = micros();

  while ((encL_count + encR_count) / 2 <= target_count)
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
      //md.setSpeeds(leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm), rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
  
  // reset PID
  leftPIDController.resetPID();
  rightPIDController.resetPID();
  // reset encoder ticks
  resetEnc();
}

void rotateRight2(double angle) // doesn't really work but I'll leave it here
{
  //Store current encoder value
  int fLval = encL_count;
  int fRval = encR_count;
  //4.8147 exact multiplier
  // Calculate target number of ticks to travel the distance,
  // reduce tEncodeVal by no. ticks needed for braking
  double L_tEncodeVal = fLval + angle * 4.42; //4.46; for brakes: //- 35;  // 38
  double R_tEncodeVal = fRval + angle * 4.42; //4.46; for brakes: //- 36;  // 33

  // set multiplier for motor speed direction
  leftSign = -1;
  rightSign = -1;

  // reset prevTime to get more accurate timeWidth
  L_prevTime = micros();
  R_prevTime = micros();

  while ((encL_count <= L_tEncodeVal) || (encR_count <= R_tEncodeVal))
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(-rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
      //md.setSpeeds(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm), -rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
  
  // reset PID
  leftPIDController.resetPID();
  rightPIDController.resetPID();
  // reset encoder ticks
  resetEnc();
}

void rotateRight(double angle)
{
  //Store current encoder value
  int curLeftEnc = encL_count;
  int curRightEnc = encR_count;
  target_count = (encL_count + encR_count) / 2;
  //4.8147 exact multiplier
  // Calculate target number of ticks to travel the distance,
  // reduce tEncodeVal by no. ticks needed for braking
  // Every degree takes (1/360 *58.11 /18.84956 * 562.25) = 4.8147
  // check if either motor reached the target number of ticksif (angle <=90)
  if (angle <= 50)  // for 45 deg
    target_count += angle * 4.27;
  else if (angle <= 90)
    target_count += angle * 4.41; // 4.42 for paper, 4.41 for arena
  else if ( 90 < angle <= 180)
    target_count += angle * 4.54;

  // set multiplier for motor speed direction
  leftSign = -1;
  rightSign = -1;

  // reset prevTime to get more accurate timeWidth
  L_prevTime = micros();
  R_prevTime = micros();

  while ((encL_count + encR_count) / 2 <= target_count)
  {
    if (PID::checkPIDCompute()) {
      md.setM1Speed(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
      md.setM2Speed(-rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
      //md.setSpeeds(-leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm), -rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
    }
  }
  md.setBrakes(BRAKE_L, BRAKE_R);
  
  // reset PID
  leftPIDController.resetPID();
  rightPIDController.resetPID();
  // reset encoder ticks
  resetEnc();
}

#define THRESHOLD 0.5

// Robot self-calibration: wall alignment
void alignToLeftWall() {
  double dist_S1, dist_S2, difference;
  bool align = true;
  
  while (align) {
    dist_S1 = left_S1.getDistance();
    dist_S2 = left_S2.getDistance();
    difference = dist_S1 - dist_S2;
    
    // abs: is a macro, so should be efficient
    if (abs(difference) < THRESHOLD) {
      md.setBrakes(400, 400);
      align = false;
    } else if (PID::checkPIDCompute()) {  // continue aligning
      if (difference > 0) { // tilted to the right; turn left
        md.setSpeeds(leftPIDController.computePID(calculateRpm(L_timeWidth), alignmentTargetRpm),
          rightPIDController.computePID(calculateRpm(R_timeWidth), alignmentTargetRpm));
      } else {  // tilted to the left; turn right
        md.setSpeeds(-leftPIDController.computePID(calculateRpm(L_timeWidth), alignmentTargetRpm),
          -rightPIDController.computePID(calculateRpm(R_timeWidth), alignmentTargetRpm));
      }
    }
  }
}

void alignToFrontWall_Left() {
  // use front-left (D3) and front-mid (D2) for alignment
  double dist_D2, dist_D3, difference;
  bool align = true;
  
  while (align) {
    dist_D2 = front_D2.getDistance();
    dist_D3 = front_D3.getDistance();

    difference = dist_D2 - dist_D3;
    
    // abs: is a macro, so should be efficient
    if (abs(difference) < THRESHOLD) {
      md.setBrakes(400, 400);
      align = false;
    } else if (PID::checkPIDCompute()) {  // continue aligning
      if (difference > 0) { // tilted to the right; turn left
        md.setSpeeds(leftPIDController.computePID(calculateRpm(L_timeWidth), alignmentTargetRpm),
          rightPIDController.computePID(calculateRpm(R_timeWidth), alignmentTargetRpm));
      } else {  // tilted to the left; turn right
        md.setSpeeds(-leftPIDController.computePID(calculateRpm(L_timeWidth), alignmentTargetRpm),
          -rightPIDController.computePID(calculateRpm(R_timeWidth), alignmentTargetRpm));
      }
    }
  }
}

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
}

void loop() {

  // main robot system loop
  //robotSystem_loop();

 // move forward/rotate in small units
//  for (int i = 0; i < 4; i++) {
//    // change angle target depending on surface?
//    //rotateLeft2(20);
//    //rotateRight(90);
//    moveForward(50);
//    leftPIDController.resetPID();
//    rightPIDController.resetPID();
//    delay(2000);
//  }

  // test PID/reading IR sensor data
//  testInLoop_motorsPID();
  testInLoop_readingIR();
}

void robotSystem_loop() {

  if (Serial.available() > 0) { // new command
    // read incoming line
    input = Serial.readString();

    // read characters to determine command
    char command = input.charAt(0);
    if (command == 'M') { // move
      // read next character for no. units to move
      //char numUnits = input.charAt(1);
      //moveForward((numUnits - '0') * 10);

      // check remaining characters as it's possible to be commanded to move >=10 units
      input.trim();
      int numUnits = input.substring(1).toInt();
      moveForward(numUnits * 10);

      // TODO: compare the performance of using String toInt() instead      

      // TODO: acknowledge the command?
      Serial.write("R\n");
      //Serial.write(command);
      //Serial.write(numUnits);
    }
    else if (command == 'T') {  // turn/rotate
      // read next character for what turn to make
      char turnBy = input.charAt(1);


      switch (turnBy) {
        case 'L': // turn left by 90 degrees
          rotateLeft(90);
          break;
        case 'R': // turn right by 90 degrees
          rotateRight(90);
          break;
        case 'B': // turn 180 degrees
          rotateLeft(180);
          break;
      }
      
      // TODO: acknowledge the command?
      Serial.write("R\n");
      //Serial.write(command);
      //Serial.write(turnBy);
    }
    else if (command == 'C') {  // calibrate
      // TODO: determine logic for robot self-calibration

    }
  } // if Serial.available() end
  /*else {  // TODO: should this even be here lmao
    // TODO: change to average ticks? ((encL_count + encR_count)/2 <= target_count)
    if ((encL_count <= target_count) || (encR_count <= target_count))
    {
      // TODO: can PID be ISR lol
      if (PID::checkPIDCompute()) {
        md.setM1Speed(leftSign * leftPIDController.computePID(calculateRpm(L_timeWidth), targetRpm));
        md.setM2Speed(rightSign * rightPIDController.computePID(calculateRpm(R_timeWidth), targetRpm));
      }

      // TODO: Send IR sensor & motor ticks data at some point?
      //sendEncoderTicks();
      //sendIRSensorsReadings();
    }
    // movement completed
    md.setBrakes(BRAKE_L, BRAKE_R);
    // POTENTIAL TODO: check if robot should self-calibrate
    }*/
}

void sendEncoderTicks() {
  int encL = encL_count;
  int encR = encR_count;
  Serial.write("Ticks,");
  Serial.write(encL);
  Serial.write(",");
  Serial.write(encR);
  Serial.write("\n");
}

typedef union {
 double floatingPoint;
 byte binary[4];
} binaryFloat;

void sendIRSensorsReadings() {

  double dist_D1 = front_D1.getDistance();
  double dist_D2 = front_D2.getDistance();
  double dist_D3 = front_D3.getDistance();
  double dist_S1 = left_S1.getDistance();
  double dist_S2 = left_S2.getDistance();
  double dist_LR = right_long.getDistance();

  byte* ptr_dist_D1 = (byte*) &dist_D1;
  byte* ptr_dist_D2 = (byte*) &dist_D2;
  byte* ptr_dist_D3 = (byte*) &dist_D3;
  byte* ptr_dist_S1 = (byte*) &dist_S1;
  byte* ptr_dist_S2 = (byte*) &dist_S2;
  byte* ptr_dist_LR = (byte*) &dist_LR;

  //binaryFloat

//  Serial.write("IR,");
//  Serial.write(ptr_dist_D1, 4);
//  Serial.write(",");
//  Serial.write(ptr_dist_D2, 4);
//  Serial.write(",");
//  Serial.write(ptr_dist_D3, 4);
//  Serial.write(",");
//  Serial.write(ptr_dist_S1, 4);
//  Serial.write(",");
//  Serial.write(ptr_dist_S2, 4);
//  Serial.write(",");
//  Serial.write(ptr_dist_LR, 4);
//  Serial.write('\n');

// TEMPORARY DUMMY VALUES
Serial.write("IR,10.0,10.0,10.0,10.0,10.0,10.0\n");
}

void testInLoop_readingIR() {
  //delay(100);

  //Serial.println(front_D1.getDistance());
  // Serial.println(front_D2.getDistance());
  Serial.print("Side, front: ");
  Serial.println(left_S2.getDistance());

//    Serial.print("Front, right: ");
//    Serial.println(front_D1.getDistance());
//    Serial.print("Front, middle: ");
//    Serial.println(front_D2.getDistance());
  //Serial.println(left_S1.getDistance());
  //Serial.println(left_S2.getDistance());
  

  //Serial.println(front_D3.getDistance());
  //Serial.println(left_S1.getDistance());
  //Serial.println(left_S2.getDistance());
//  Serial.println(right_long.getDistance());
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
    //Serial.print(leftPIDController.getLastValue());
    //Serial.write(",");
    //Serial.print(leftPIDController.getSumError());
    //Serial.write(",");
    //Serial.print(rightPIDController.getLastValue());
    //Serial.write(",");
    //Serial.println(rightPIDController.getSumError());

    //md.setM1Speed(-350);
    //md.setM2Speed(350);
    // update motor speed with PID controller
    md.setM1Speed(-leftPIDController.computePID(L_rpm, targetRpm));
    md.setM2Speed(rightPIDController.computePID(R_rpm, targetRpm));
    //md.setSpeeds(-leftPIDController.computePID(L_rpm, targetRpm), rightPIDController.computePID(R_rpm, targetRpm));
  }
}
