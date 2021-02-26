#ifndef AlignAndCheck_H
#define AlignAndCheck_H
#include "SharpIR.h"

#define FRONT_INITIAL_EXPECTED_DIST 4.0
#define STATE_DIFFGT0 1
#define STATE_DIFFLT0 2
#define THRESHOLD 0.05
#define HS_THRESHOLD 0.8  // high-speed threshold

extern double D1_EXPECTED_DIST; 
extern double D2_EXPECTED_DIST;
extern double D3_EXPECTED_DIST;
extern double S1_EXPECTED_DIST;
extern double S2_EXPECTED_DIST;

void checkForCrashCalibration();
void checkForAlignmentCalibration_initial();
void checkForAlignmentCalibration();
void alignBack_FrontMid();
void alignToLeftWall();
void alignToFrontWall_Left();
void alignToFrontWall_Right();
void alignBack_Front(SharpIR::sensorCode sensor, bool fast);
void alignForward_Front(SharpIR::sensorCode sensor, bool fast);
void initialGridCalibration();


#endif
