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

  //return median;

  switch( sensorType )
  {
  case D1:
      // check out of range
      if(median > 639) return 7;
      else if (median >= 601) // 7-8cm
        distance = 14.06667 + 1629.102/(median-869.5333);
      else if (median >= 347) // 8-15cm
        distance = -5.67432 + 10198.92/(median+146.005);
      else if (median >= 125) // 15-45cm
        distance = -1.37507 + 5666.653/(median-2.777881);
      else if (median >= 110) // 45-50cm
        distance = -6.1679 + 7732.175/(median+27.5616);
      else return 51; // median < 110, out of range
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
      if (median > 635) return 7;
      else if (median >= 591) // 7-8cm
        distance = 21.43182 - 0.02273*median;
      else if (median >= 168) // 8-35cm
        distance = -1.28379 + 5321.748/(median-21.49596);
      else if (median >= 142) // 35-40cm
        distance = 61.20559 + 2894.61/(median-278.5809);
      else if (median >= 114) // 40-50cm
        distance = 90.81596 - 0.31157*median;
      else return 51; // median < 114, out of range
      return distance - D3_OFFSET;

  case S1:  // GO
      // check out of range
      if (median > 622) return 7;
      else if (median >= 125) // 7-40cm
        distance = -1.82125 + 5790.725/(median+12.6569);
      else if (median >= 101) // 40-47.5cm
        distance = 79.0625 - 0.3125*median;
      else return 48; // median < 101, out of range
      return distance - S1_OFFSET;

  case S2:
      // check out of range
      if (median > 614) return 8;
      else if (median >= 142) // 8-40cm
        distance = -1.61521 + 5974.987/(median+2.26629);
      else if (median >= 114) // 40-50cm
        distance = 90.55131 - 0.35561*median;
      else return 51; // median < 114, out of range
      return distance - S2_OFFSET;
      
  case LR:
      // check median ranges
      if(median > 550) return 13; // out of range
      else if (median >= 539) // 13-15cm
        distance = 123 - 0.2*median;
      else if (median >= 499) // 15-20cm
        distance = 65.7006 + 18567.84/(median-905.1389);
      else if (median >= 398)  // 20-30cm
        distance = 69.42099 - 0.09901*median;
      else if (median >= 353) // 30-35cm
        distance = 76.17186 - 0.1162*median;
      else if (median >= 160) // 35-80cm
        distance = -0.12527 + 12186.82/(median-7.023576);
      else return 80;  // median < 160, out of range
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
