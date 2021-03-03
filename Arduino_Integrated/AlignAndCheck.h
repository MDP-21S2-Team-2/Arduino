#ifndef AlignAndCheck_H
#define AlignAndCheck_H
#include "SharpIR.h"

#define STATE_DIFFGT0 1
#define STATE_DIFFLT0 2
#define THRESHOLD 0.05  // for alignment when front/side is tilted
#define FLUC_THRESHOLD 0.2  // for sensor fluctuation threshold
#define SIDE_THRESHOLD 0.125
#define HS_THRESHOLD 0.8  // high-speed threshold

#define FRONT_INITIAL_EXPECTED_DIST 5.0
#define FRONT_1GRID_DIST 5.0
#define FRONT_1GRID_START 1.8
#define FRONT_1GRID_END 10.5
#define LEFT_1GRID_DIST 5.0
#define LEFT_1GRID_START 2.0
#define LEFT_1GRID_END 9.5
#define RIGHT_1GRID_DIST 5.0
#define RIGHT_1GRID_START 2.0
#define RIGHT_1GRID_END 9.5

void checkForCrashCalibration();
void checkForAlignmentCalibration_initial();
void checkForAlignmentCalibration();
void alignToLeftWall();
void alignToFrontWall_Left();
void alignToFrontWall_Right();
void alignBack_Front(SharpIR::sensorCode sensor, bool fast, double targetDist);
void alignForward_Front(SharpIR::sensorCode sensor, bool fast, double targetDist);
void initialGridCalibration();


#endif
