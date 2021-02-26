#include <EnableInterrupt.h>
#include "PID.h"
#include "SharpIR.h"
#include "Movement.h"
#include "AlignAndCheck.h"
#include "Sensors.h"

// pins for the motors' encoder channels
#define LeftMotorA 11 // E2A
#define RightMotorA 3 // E1A

// Setting Target RPM 125 RPM
#define targetRpm 125.0
#define MOVE_OFFTICKS 24
// 100 RPM
//#define targetRpm 100.0
//#define MOVE_OFFTICKS 20

const float alpha = 0.3;
const float alphaInv = 0.7;

// Serial communication
String input;

void motorR_ISR() {
  encR_count++;
  encR_curr_count++;
  //For every 10 encoder count
   if (encR_curr_count == 1)
    {
      R_prevTime = micros();    
    }
    else if (encR_curr_count == 6)
    {
      R_currTime = micros();
      R_timeWidth = alpha * (R_currTime - R_prevTime) + alphaInv * R_timeWidth;
      encR_curr_count = 0;
      R_prevTime = R_currTime;
    }
}

void motorL_ISR() {
  encL_count++;
  encL_curr_count++;
   if (encL_curr_count == 1)
    {
      L_prevTime = micros();    
    }
    else if (encL_curr_count == 6)
    {
      L_currTime = micros();
      L_timeWidth = alpha * (L_currTime - L_prevTime) + alphaInv * L_timeWidth;
      encL_curr_count = 0;
      L_prevTime = L_currTime;
    }
}
// PID controller for each motor
// parameter list: P, I, D, Imax, Imin

// 24 Feb target speed: 125 6.26V 1y1w
//PID leftPIDController(3.8, 2.6, 5.2, 130.0, -130); // red
//PID rightPIDController(3.55, 2.6, 5.3, 130.0, -130.0); // right starts up faster
// 25 Feb target speed: 125 6.26V 1y1w
extern PID leftPIDController; // red
extern PID rightPIDController; // right starts up faster


// Distance Function
//double leftWheelDiameter = 6.0;   // in cm
//double rightWheelDiameter = 6.0;  // in cLLm
//Wheel to wheel distance = 18.5cm
//Circumference of whole robot m= 58.11cm
// 90 degree = 14.5275cm


void setup() {
  // put your setup code here, to run once:
  //init encoder pins
  //(default is input so technically not needed)
  //pinMode(LeftMotorA, INPUT);
  //pinMode(RightMotorA, INPUT);

  // enable interrupts for calculating motor rpm
  enableInterrupt(LeftMotorA, motorL_ISR, RISING);
  enableInterrupt(RightMotorA, motorR_ISR, RISING);

  // init Serial
  Serial.begin(115200);
  // init motor drivers
  md.init();

  // left
  //md.setM1Speed(-400);
  // right
  //md.setM2Speed(400);

  // set starting times
//  L_prevTime = micros();
//  R_prevTime = L_prevTime;
}



void robotSystem_loop() {

  if (Serial.available() > 0) { // new command
    // read incoming line
    input = Serial.readString();

    // read characters to determine command
    char command = input.charAt(0);
    
    switch (command) {  // TODO: a switch case could be more efficient if the characters are sequential?
    case 'M': // move <=10 units
      {
        // read next character for no. units to move
        char numUnits = input.charAt(1);
        moveForward(numUnits - '0');
        
        // TODO: acknowledge the command?
        Serial.write("R\n");
      }
      break;
    case 'F': // move >=11 units
      {
        // read next character for no. units to move
        char numUnits = input.charAt(1);
        moveForward(numUnits - '%'); // numUnits - '0' + 11
        
        // TODO: acknowledge the command?
        Serial.write("R\n");
      }
      break;
    case 'L': // turn left 90
      rotateLeft(90);
      // TODO: acknowledge the command?
      Serial.write("R\n");
      break;
    case 'R': // turn right 90
      rotateRight(90);
      // TODO: acknowledge the command?
      Serial.write("R\n");
      break;
    case 'B': // turn 180
      rotateLeft(180);
      // TODO: acknowledge the command?
      Serial.write("R\n");
      break;
    case 'C': // initial calibration in starting grid
      initialGridCalibration();
      break;
      
    default:  // do nothing
      break;
    }
    
  } // if Serial.available() end
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
  Serial.write(ptr_dist_D1, 4);
  Serial.write(",");
  Serial.write(ptr_dist_D2, 4);
  Serial.write(",");
  Serial.write(ptr_dist_D3, 4);
  Serial.write(",");
  Serial.write(ptr_dist_S1, 4);
  Serial.write(",");
  Serial.write(ptr_dist_S2, 4);
  Serial.write(",");
  Serial.write(ptr_dist_LR, 4);
  Serial.write('\n');

  // TEMPORARY DUMMY VALUES
  //Serial.write("IR,10.0,10.0,10.0,10.0,10.0,10.0\n");
}

void testInLoop_readingIR() {
  //delay(100);

  Serial.print("Front Right (D2): ");
  Serial.print(front_D2.getDistance());
  Serial.print(" | Front Mid (D1): ");
  Serial.print(front_D1.getDistance());
  Serial.print(" | Front Left (D3): ");
  Serial.println(front_D3.getDistance());

//  Serial.print("Side, front: ");
//  Serial.print(left_S1.getDistance());
//  Serial.print(" | Side, back: ");
//  Serial.println(left_S2.getDistance());
//
//  Serial.print("Right Long: ");
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
void loop() {

//  testInLoop_motorsPID();
//  testInLoop_readingIR();
    robotSystem_loop();
}
