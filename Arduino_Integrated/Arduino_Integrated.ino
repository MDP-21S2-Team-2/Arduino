#include <EnableInterrupt.h>
#include "Motors.h"
#include "Sensors.h"
#include "Movement.h"
#include "Alignment.h"

// robot configuration
bool enableAlignAfterMove_FP = true; // enabled by default
bool enableEbrakes_FP = true; // enabled by default
bool enableMoveInParts = true; // enabled by default
int numParts_FP = 2;  // max no. units to move at a time
int numParts_W = 4; // max no. units to move at a time

bool stopRunning = false; // stop running (backup)

void setup() {
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
}

// move a no. of units at a time
// targetUnits = 0: 1 unit, targetUnits = 1: 2 units, ... targetUnits = 3: 4 units, targetUnits = 4: 5 units
void moveInParts(int targetUnits, bool enableEbrakes, int additionalTicks = 0) {
  int ticksPerUnit = additionalTicks / targetUnits;
  while (targetUnits >= numParts_FP) {
    moveForward(numParts_FP - 1, enableEbrakes, true, numParts_FP*ticksPerUnit);
    targetUnits -= numParts_FP;
    if (enableAlignAfterMove_FP) {
      delay(120);
      //checkAlignmentAfterCommand_FP();
      checkAlignmentAfterMove();
    }
    delay(60);
  }
  if (targetUnits > 0) {  // still have a smaller part to move
    moveForward(targetUnits - 1, enableEbrakes, true, targetUnits*ticksPerUnit);
    delay(20);
  }
}

void robotSystem_loop() {

  if (stopRunning)
    return;

  if (Serial.available() > 0) { // new command
    // read incoming line
    // read characters to determine command
    char command = Serial.read();
    if ((command == '\r') || (command == '\n')) // newline, ignore
      return;

    switch (command) {
      case 'M': // move <=10 units
        {
          // read next character for no. units to move
          while (Serial.available() == 0);  // wait for next character
          char numUnits = Serial.read();
#ifdef EXPLORATION_MODE
          if (enableMoveInParts) {
            moveInParts(numUnits - '0' + 1, enableEbrakes_FP);
          }
          else
#endif
            moveForward(numUnits - '0', enableEbrakes_FP);
#ifdef EXPLORATION_MODE
          delay(120);
          checkAlignmentAfterMove();
          delay(60);
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
#ifdef EXPLORATION_MODE
          if (enableMoveInParts) {
            moveInParts(numUnits - '&' + 1, enableEbrakes_FP);
          }
          else
#endif
            moveForward(numUnits - '&', enableEbrakes_FP); // numUnits - '0' + 10
#ifdef EXPLORATION_MODE
          delay(120);
          checkAlignmentAfterMove();
          delay(60);
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
#ifdef EXPLORATION_MODE
          if (enableMoveInParts) {
            moveInParts(numUnits - '0' + 1, enableEbrakes_FP);
          }
          else
#endif
          moveForward(numUnits - '0', enableEbrakes_FP);
#ifdef EXPLORATION_MODE
          delay(120);
          checkAlignmentAfterMove();
          delay(60);
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
#ifdef EXPLORATION_MODE
          if (enableMoveInParts) {
            moveInParts(numUnits - '&' + 1, enableEbrakes_FP);
          }
          else
#endif
            moveForward(numUnits - '&', enableEbrakes_FP); // numUnits - '0' + 10
#ifdef EXPLORATION_MODE
          delay(120);
          checkAlignmentAfterMove();
          delay(60);
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
          //moveForward();
          bool stopped = false;
          int unitsMoved = 0;
          int currUnitsMoved = 0;
          while (!stopped) {
            //stopped = moveForward(numParts_W - 1, true, false, 0, true);
            stopped = moveForward_W(numParts_W - 1, &currUnitsMoved );
            if (stopped)
              unitsMoved += computeUnitsMoved();
            else {
              unitsMoved += numParts_W;
              if (currUnitsMoved  < numParts_W)
                sendRightSensorReadings();
              // check alignment & calibration
              delay(120);
              checkAlignmentAfterMove();
              delay(60);
            }
          }
          // send no. units moved
          sendUnitsMoved(unitsMoved);
#ifdef EXPLORATION_MODE
          delay(120);
          checkAlignmentAfterMove();
          delay(60);
          // send sensor readings
          sendIRSensorsReadings();
#else // FP
          Serial.write("K\n");
          delay(60);    
#endif
        }
        break;

      case 'S': // move with additional ticks to overcome skid
        {
          // read next character for no. units to move
          while (Serial.available() == 0);  // wait for next character
          char numUnits = Serial.read();
          int additionalTicks = 0;  // ADDITIONAL TICKS
          if (enableMoveInParts) {
            moveInParts(numUnits - '0' + 1, enableEbrakes_FP, additionalTicks);
          }
          else
            moveForward(numUnits - '0', enableEbrakes_FP, true, additionalTicks);
#ifdef EXPLORATION_MODE
          delay(120);
          checkAlignmentAfterMove();
          delay(60);
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

      case 'L': // turn left 90
        rotateLeft(90);
#ifdef EXPLORATION_MODE
        delay(130);
        checkAlignmentAfterRotate();
        delay(60);
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
        delay(130);
        checkAlignmentAfterRotate();
        delay(60);
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
        delay(140);
        checkAlignmentAfterRotate();
        delay(60);
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

      case 'm': // partial step forward - not used
        moveForward_custom(4.0, true);
#ifdef EXPLORATION_MODE
        delay(110);
        checkAlignmentAfterRotate();
        delay(60);
        // send sensor readings
        sendIRSensorsReadings();
#endif
        break;

      case 'b': // partial step backward - not used
        moveBackward_custom(4.0);
#ifdef EXPLORATION_MODE
        delay(110);
        checkAlignmentAfterRotate();
        delay(60);
        // send sensor readings
        sendIRSensorsReadings();
#endif
        break;

      case 'C': // initial calibration in starting grid
        initialGridCalibration();
        stopRunning = false;
#ifdef EXPLORATION_MODE
        // send sensor readings
        sendIRSensorsReadings();
#else // FP
        // acknowledge the command
        Serial.write("K\n");
#endif
        break;

      case 'T': // stop running (backup)
        stopRunning = true;
        break;

      default:  // do nothing
        break;
    }

  } // if Serial.available() end
}

// For testing only
void testInLoop_readingIR() {
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

// For testing only
void testInLoop_motorsPID() {
  if (PID::checkPIDCompute()) {
    //Serial.println(R_timeWidth);
    //Serial.println(R_timeWidth);

    double R_rpm = calculateRpm(R_timeWidth);
    double L_rpm = calculateRpm(L_timeWidth);

    Serial.print(R_rpm);
    Serial.print(",");
    Serial.println(L_rpm);

    // update motor speed with PID controller
    md.setM1Speed(-leftPIDController.computePID(L_rpm, targetRpm));
    md.setM2Speed(rightPIDController.computePID(R_rpm, targetRpm));
  }
}

void loop() {
//  testInLoop_motorsPID();
//  testInLoop_readingIR();
  robotSystem_loop();
}
