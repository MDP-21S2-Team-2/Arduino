#include "SharpIR.h"

double SharpIR::getDistance() {

  //return analogRead(pin);

  int readings[NUM_SAMPLES] = {};
  double distance;  // in cm
  int median;


  // 2800-2808 microseconds for 25 readings
  for (int i=0; i< NUM_SAMPLES; i++) {
    // Read analog value
    readings[i] = analogRead(pin);
  }
  
  // regular median filter
#ifdef MEDIAN_FILTER
  // get median reading from analog pin
  sort(readings, NUM_SAMPLES); // sort the readings
  median = readings[HALF_NUM_SAMPLES]; // return the median reading
  //Serial.println("Median");
#endif
#ifdef MEDOFMEDIANS_FILTER
  median = medianOfMedians(readings, NUM_SAMPLES);  // 28, 40-56 microseconds
  //Serial.println("Median of medians");
#endif
  return median;
  switch( sensorType )
  {
  case D1:
      // check out of range
      if(median > 639) return 7;
      else if(median < 110) return 51;
      distance = -2.2771 + 6159.08/(median+6.781);
      return distance - D1_OFFSET;

  case D2:
      // check out of range
      if(median > 586) return 7;
      else if(median < 109) return 51;
      distance = -0.87161 + 5127.804/(median-8.158281);
      return distance - D2_OFFSET;

  case D3:
      // check out of range
      if(median > 635) return 7;
      else if(median < 114) return 51;
      distance = -3.61655 + 6829.367/(median+12.4999);
      return distance - D3_OFFSET;

  case S1:  // GO
      // check out of range
      if(median > 622) return 7;
      else if(median < 101) return 51;
      distance = -2.5396 + 6251.4/(median+22.56);
      return distance - S1_OFFSET;

  case S2:
      // check out of range
      if(median > 614) return 8;
      else if(median < 114) return 51;
      distance = -1.314 + 5779.5093/(median-2.45285);
      return distance - S2_OFFSET;
      
  case LR:
      // check out of range
      if(median > 540) return 15;
      else if(median < 165) return 77;
      distance = -24.0575 + 25654.8826/(median+91.9346);
      return distance - LR_OFFSET;
  }
}

#define NUM_MEDIANS 5 // NUM_SAMPLES / 5
#define HALF_NUM_MEDIANS 2  // NUM_MEDIANS / 2

int SharpIR::medianOfMedians(int a[], int size){
  int ans;
  //int numMedians = size / 5;
  int medians[NUM_MEDIANS] = {};
  for(int i = 0; i < NUM_MEDIANS; ++i){
    partialSort(a, i * 5, i * 5 + 4);
    medians[i] = a[i * 5 + 2];
  }
  sort(medians, NUM_MEDIANS); // sort the medians
  return medians[HALF_NUM_MEDIANS]; // return the median of medians
}

// Sort a partial arrays
void SharpIR::partialSort(int a[], int min, int max) {
  int t;
  bool flag = true;
  for(int i=min; i<max; i++) {
    for(int o=min; o<(max-i); o++) {
      if(a[o] > a[o+1]) {
        // swap elements
        t = a[o];
        a[o] = a[o+1];
        a[o+1] = t;
        flag = false;
      }
    }
    if (flag) break;  // did not have to swap in the first iteration, so everything is in order
  }
}

// Sort an array
void SharpIR::sort(int a[], int size) {
  int t;
  bool flag = true;
  for(int i=0; i<(size-1); i++) {
    for(int o=0; o<(size-(i+1)); o++) {
      if(a[o] > a[o+1]) {
        // swap elements
        t = a[o];
        a[o] = a[o+1];
        a[o+1] = t;
        flag = false;
      }
    }
    if (flag) break;  // did not have to swap in the first iteration, so everything is in order
  }
}
