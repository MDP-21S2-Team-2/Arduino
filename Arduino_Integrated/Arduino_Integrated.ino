#include <EnableInterrupt.h>
#include "Motors.h"
#include "Sensors.h"
#include "Movement.h"
#include "Alignment.h"

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
    // read characters to determine command
    char command = Serial.read();
    if ((command == '\r') || (command == '\n')) // newline, ignore
      return;

    switch (command) {  // TODO: a switch case might be more efficient if the characters are sequential?
      case 'M': // move <=10 units
        {
          // read next character for no. units to move
          while (Serial.available() == 0);  // wait for next character
          char numUnits = Serial.read();
          moveForward(numUnits - '0', false);
#ifdef EXPLORATION_MODE
          delay(500);
          checkAlignmentAfterMove();
          delay(500);
          // send sensor readings
          sendIRSensorsReadings();
#else // FP
          // acknowledge the command
          Serial.write("K\n");
          delay(80);
#endif
        }
        break;
        
      case 'F': // move >=11 units
        {
          // read next character for no. units to move
          while (Serial.available() == 0);  // wait for next character
          char numUnits = Serial.read();
          moveForward(numUnits - '&', false); // numUnits - '0' + 10
#ifdef EXPLORATION_MODE
          delay(500);
          checkAlignmentAfterMove();
          delay(500);
          // send sensor readings
          sendIRSensorsReadings();
#else // FP
          // acknowledge the command
          Serial.write("K\n");
          delay(80);
#endif
        }
        break;

      case 'E': // move <=10 units, emergency brakes enabled
        {
          // read next character for no. units to move
          while (Serial.available() == 0);  // wait for next character
          char numUnits = Serial.read();
          moveForward(numUnits - '0', true);
#ifdef EXPLORATION_MODE
          delay(500);
          checkAlignmentAfterMove();
          delay(500);
          // send sensor readings
          sendIRSensorsReadings();
#else // FP
          // acknowledge the command
          Serial.write("K\n");
          delay(80);
#endif
        }
        break;

      case 'D': // move >=11 units, emergency brakes enabled
        {
          // read next character for no. units to move
          while (Serial.available() == 0);  // wait for next character
          char numUnits = Serial.read();
          moveForward(numUnits - '&', true); // numUnits - '0' + 10
#ifdef EXPLORATION_MODE
          delay(500);
          checkAlignmentAfterMove();
          delay(500);
          // send sensor readings
          sendIRSensorsReadings();
#else // FP
          // acknowledge the command
          Serial.write("K\n");
          delay(80);
#endif
        }
        break;
        
      case 'L': // turn left 90
        rotateLeft(90);
#ifdef EXPLORATION_MODE
        delay(500);
        checkAlignmentAfterRotate();
        delay(500);
        // send sensor readings
        sendIRSensorsReadings();
#else // FP
        // acknowledge the command
        Serial.write("K\n");
        delay(80);
#endif
        break;
        
      case 'R': // turn right 90
        rotateRight(90);
#ifdef EXPLORATION_MODE
        delay(500);
        checkAlignmentAfterRotate();
        delay(500);
        // send sensor readings
        sendIRSensorsReadings();
#else // FP
        // acknowledge the command
        Serial.write("K\n");
        delay(80);
#endif
        break;
      case 'B': // turn 180
        rotateLeft(180);
#ifdef EXPLORATION_MODE
        delay(500);
        checkAlignmentAfterRotate();
        delay(500);
        // send sensor readings
        sendIRSensorsReadings();
#else // FP
        // acknowledge the command
        Serial.write("K\n");
        delay(80);
#endif
        break;
        
      case 'C': // initial calibration in starting grid
        initialGridCalibration();
#ifdef EXPLORATION_MODE
        // send sensor readings
        sendIRSensorsReadings();
#else // FP
        // acknowledge the command
        Serial.write("K\n");
#endif
        break;

      default:  // do nothing
        break;
    }

  } // if Serial.available() end
}

void testInLoop_readingIR() {
  //delay(100);

//  Serial.print("Front Right (D2): ");
//  Serial.print(front_D2.getDistance());
//  Serial.print(" | Front Mid (D1): ");
//  Serial.print(front_D1.getDistance());
//  Serial.print(" | Front Left (D3): ");
//  Serial.println(front_D3.getDistance());

    Serial.print("Side, front: ");
    Serial.print(left_S1.getDistance());
    Serial.print(" | Side, back: ");
    Serial.println(left_S2.getDistance());

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

bool runProgram = true;
void loop() {
//  sendIRSensorsReadings();
//  delay(500);
  //  testInLoop_motorsPID();
//    testInLoop_readingIR();
  robotSystem_loop();
//  initialGridCalibration();
//  delay(5000);
  //    delay(1000);
////  if (runProgram) {
      //delay(500);
//    for (int i = 0; i < 3; ++i) {
//      moveForward(2);
//      delay(500);
//    };

//    runProgram = false;
//  }
}
