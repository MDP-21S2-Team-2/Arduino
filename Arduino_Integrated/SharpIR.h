/*
 * SharpIR
 * Modified SharpIR library for acquisition of distance data from analog Sharp IR sensors
 * Original library: SharpIR Ver 2.0.1 by Giuseppe Masino (HackerInside)
 */

#ifndef SHARP_IR_H
#define SHARP_IR_H

#define MEDIAN_FILTER
//#define MEDOFMEDIANS_FILTER

#define NUM_SAMPLES 25  // for median filter: read odd no. times
#define HALF_NUM_SAMPLES 12 // index of median

// physical sensor offsets
#define D1_OFFSET 0
#define D2_OFFSET 0
#define D3_OFFSET 0
#define S1_OFFSET 0
#define S2_OFFSET 0
#define LR_OFFSET 0

#include <Arduino.h>

class SharpIR {
  // private variables
  uint32_t lastTime = 0;

  // protected variables
  protected:
    uint8_t sensorType, pin;

  // public methods/variables
  public:
    using sensorCode = const uint8_t;

    // constructor
    /// default for all pins is input, thus not required to configure
    SharpIR( sensorCode _sensorType , uint8_t _sensorPin ) :
      sensorType( _sensorType ) , pin( _sensorPin ) {}

    // get distance from analog input
    double getDistance(bool avoidBurstRead);

    // median of medians implementation from archery2000 @ https://github.com/guillaume-rico/SharpIR/
    void partialSort(int a[], int min, int max);
    int medianOfMedians(int a[], int size);
    void sort(int a[], int size);
    
    // sensor types
    static sensorCode D1 = 1; // short range, right
    static sensorCode D2 = 2; // short range, middle ("bad")
    static sensorCode D3 = 3; // short range, left ("good" until 30cm)
    static sensorCode S1 = 4; // short range, front ("good", "G0")
    static sensorCode S2 = 5; // short range, back
    static sensorCode LR = 6; // long range
};

#endif
