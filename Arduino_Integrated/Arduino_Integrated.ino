#include <EnableInterrupt.h>
#include "Motors.h"
#include "Sensors.h"
#include "Movement.h"
#include "Alignment.h"

// robot configuration (intended for FP use)
bool enableAlignAfterMove_FP = true; // enabled by default
bool enableEbrakes_FP = true; // enabled by default
bool enableMoveInParts = false; // disabled by default
int numParts_FP = 3;  // max no. units to move at a time

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
  //  md.setM1Speed(-300);
  // right
  //  md.setM2Speed(270);

  // set starting times
  //  L_prevTime = micros();
  //  R_prevTime = L_prevTime;
}

// move a no. of units at a time
// targetUnits = 0: 1 unit, targetUnits = 1: 2 units, ... targetUnits = 3: 4 units, targetUnits = 4: 5 units
void moveInParts(int targetUnits, bool enableEbrakes) {
  while (targetUnits >= numParts_FP) {
    moveForward(numParts_FP - 1, enableEbrakes);
    targetUnits -= numParts_FP;
    if (enableAlignAfterMove_FP) {
      delay(60);
      //            checkAlignmentAfterCommand_FP();
      checkAlignmentAfterMove();
    }
    delay(60);
  }
  if (targetUnits > 0) {  // still have a smaller part to move
    moveForward(targetUnits - 1, enableEbrakes);
  }
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
          if (enableMoveInParts) {
            moveInParts(numUnits - '0' + 1, enableEbrakes_FP);
          }
          else
            moveForward(numUnits - '0', enableEbrakes_FP);
#ifdef EXPLORATION_MODE
          delay(70);
          checkAlignmentAfterMove();
          delay(70);
          // send sensor readings
          sendIRSensorsReadings();
#else // FP
          if (enableAlignAfterMove_FP) {
            delay(60);
            checkAlignmentAfterCommand_FP();
//            checkAlignmentAfterMove();
          }
          // acknowledge the command
          Serial.write("K\n");
          delay(60);
#endif
        }
        break;

      case 'F': // move >=11 units
        {
          // read next character for no. units to move
          while (Serial.available() == 0);  // wait for next character
          char numUnits = Serial.read();
          if (enableMoveInParts) {
            moveInParts(numUnits - '&' + 1, enableEbrakes_FP);
          }
          else
            moveForward(numUnits - '&', enableEbrakes_FP); // numUnits - '0' + 10
#ifdef EXPLORATION_MODE
          delay(70);
          checkAlignmentAfterMove();
          delay(70);
          // send sensor readings
          sendIRSensorsReadings();
#else // FP
          if (enableAlignAfterMove_FP) {
            delay(60);
            checkAlignmentAfterCommand_FP();
//            checkAlignmentAfterMove();
          }
          // acknowledge the command
          Serial.write("K\n");
          delay(60);
#endif
        }
        break;

      case 'E': // move <=10 units; last step - no calibrate
        {
          // read next character for no. units to move
          while (Serial.available() == 0);  // wait for next character
          char numUnits = Serial.read();
          if (enableMoveInParts) {
            moveInParts(numUnits - '0' + 1, enableEbrakes_FP);
          }
          else
          moveForward(numUnits - '0', enableEbrakes_FP);
#ifdef EXPLORATION_MODE
          delay(70);
          checkAlignmentAfterMove();
          delay(70);
          // send sensor readings
          sendIRSensorsReadings();
#else // FP
          // acknowledge the command
          Serial.write("K\n");
          delay(60);
#endif
        }
        break;

      case 'D': // move >=11 units; last step - no calibrate
        {
          // read next character for no. units to move
          while (Serial.available() == 0);  // wait for next character
          char numUnits = Serial.read();
          if (enableMoveInParts) {
            moveInParts(numUnits - '&' + 1, enableEbrakes_FP);
          }
          else
            moveForward(numUnits - '&', enableEbrakes_FP); // numUnits - '0' + 10
#ifdef EXPLORATION_MODE
          delay(70);
          checkAlignmentAfterMove();
          delay(70);
          // send sensor readings
          sendIRSensorsReadings();
#else // FP
          // acknowledge the command
          Serial.write("K\n");
          delay(60);
#endif
        }
        break;

      case 'W': // move until reach wall/obstacle
        {
          moveForward();
          int units = computeUnitsMoved();
          // send no. units moved
          sendUnitsMoved(units);
#ifdef EXPLORATION_MODE
          delay(70);
          checkAlignmentAfterMove();
          delay(70);
          // send sensor readings
          sendIRSensorsReadings();
#else // FP
          Serial.write("K\n");
          delay(60);    
#endif
        }
      break;

      case 'L': // turn left 90
        rotateLeft(90);
#ifdef EXPLORATION_MODE
        delay(70);
        checkAlignmentAfterRotate();
        delay(70);
        // send sensor readings
        sendIRSensorsReadings();
#else // FP
        if (enableAlignAfterMove_FP) {
          delay(60);
          checkAlignmentAfterCommand_FP();
//          checkAlignmentAfterRotate();
        }
        // acknowledge the command
        Serial.write("K\n");
        delay(60);
#endif
        break;

      case 'R': // turn right 90
        rotateRight(90);
#ifdef EXPLORATION_MODE
        delay(70);
        checkAlignmentAfterRotate();
        delay(70);
        // send sensor readings
        sendIRSensorsReadings();
#else // FP
        if (enableAlignAfterMove_FP) {
          delay(60);
          checkAlignmentAfterCommand_FP();
//          checkAlignmentAfterRotate();
        }
        // acknowledge the command
        Serial.write("K\n");
        delay(60);
#endif
        break;
      case 'B': // turn 180
        rotateLeft(180);
#ifdef EXPLORATION_MODE
        delay(70);
        checkAlignmentAfterRotate();
        delay(70);
        // send sensor readings
        sendIRSensorsReadings();
#else // FP
        if (enableAlignAfterMove_FP) {
          delay(60);
          checkAlignmentAfterCommand_FP();
//          checkAlignmentAfterRotate();
        }
        // acknowledge the command
        Serial.write("K\n");
        delay(60);
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

      case 'X': // testing
      {
        initialGridCalibration(); // NOTE: make initial grid calibration end towards a certain direction, if robot cannot move straight for long distances
        delay(200);
        moveForward();
        int units = computeUnitsMoved();
        // print no. ticks moved
        int ticks = encL_overshootCount * 256 + encL_count;
        Serial.print("Ticks: ");
        Serial.println(ticks);
        // send no. grids moved
        Serial.print("Units: ");
        Serial.println(units);
        delay(70);
        checkAlignmentAfterMove();
        delay(70);
      }

      default:  // do nothing
        break;
    }

  } // if Serial.available() end
}

void testInLoop_readingIR() {
  //delay(100);

  Serial.print("Front Right (D2): ");
  Serial.print(front_D2.getDistance());
  Serial.print(" | Front Mid (D1): ");
  Serial.print(front_D1.getDistance());
  Serial.print(" | Front Left (D3): ");
  Serial.println(front_D3.getDistance());

  Serial.print("Side, front: ");
  Serial.print(left_S1.getDistance());
  Serial.print(" | Side, back: ");
  Serial.println(left_S2.getDistance());

  Serial.print("Right Long: ");
  Serial.println(right_long.getDistance());
  delay(20);  // frequency = ?
  
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

    //    double R_speed = rightPIDController.computePID(R_rpm, targetRpm);
    //    double L_speed = leftPIDController.computePID(L_rpm, targetRpm);

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
//      testInLoop_motorsPID();
//      testInLoop_readingIR();
//  robotSystem_loop();
    for (int i = 0; i < 4; ++i) {
      //rotateLeft(90);
      //rotateRight(90);
      //moveForward(0, false);
      //delay(140);
      rotateLeft(180);
      delay(140);
    }
    delay(1500);

//  if (runProgram) {
//    runProgram = false;
//    moveForward(15, false);
//  }
}
