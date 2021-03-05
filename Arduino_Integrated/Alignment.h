#ifndef ALIGNMENT_H
#define ALIGNMENT_H

#include "SharpIR.h"

#define STATE_DIFFGT0 1
#define STATE_DIFFLT0 2
#define THRESHOLD 0.05  // for alignment when front is tilted
#define FLUC_THRESHOLD 0.2  // threshold for sensor readings to fluctuate
#define SIDES_DIST_THRESHOLD 1.0 // threshold for deciding how different sensor readings should be from target distance before calibrating (centralise sides)
#define SIDE_THRESHOLD 0.1  // for alignment when left side is tilted
#define HS_THRESHOLD 0.8  // high-speed threshold

#define FRONT_1GRID_DIST 5.0
#define FRONT_1GRID_START 1.8
#define FRONT_1GRID_END 10.5
#define LEFT_1GRID_DIST 6.0
#define LEFT_1GRID_START 3.0
#define LEFT_1GRID_END 9.5
#define RIGHT_1GRID_DIST 3.0
#define RIGHT_1GRID_START 2.0
#define RIGHT_1GRID_END 9.5

// checking for whether robot might crash while moving
void checkForCrash();

// checking for alignment
void checkCentralise_Front();  // check that robot is in the center of the grids using obstacles in front
void checkCentralise_Sides();  // check that robot is in the center of the grids using obstacles on the sides
void checkForTilted();  // check if robot is tilted, i.e. not straight on the grids

// alignment behaviour - rotation
void alignToLeftWall();
void alignToFrontWall(bool useLeft);
// alignment behaviour - centralise
void alignBack_Front(SharpIR::sensorCode sensor, bool fast, double targetDist);
void alignForward_Front(SharpIR::sensorCode sensor, bool fast, double targetDist);

#endif
