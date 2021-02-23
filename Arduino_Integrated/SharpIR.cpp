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
    if (median > 632)
      distance = 7.0;
    else if (median >= 532) //7-9
      distance = 13.5555+ 1493.2/(median-859.778);
    else if (median >= 340) //9-15
      distance = 0.0000581*median*median - 0.081448*median +35.93;
    else if (median >= 263) //15-20
      distance = (0.0001827*median*median)-0.175152*median + 53.42216;
    else if (median >= 180) //20-30
      distance = (0.11937)+(4929.93)/(median-15.281);
//    else if (median >= 137) //33-40  bad readings off by 1cm
//      distance = 94.9596+(-0.516238)*(median) + (0.000854)*(median)*(median);
    else if (median >= 153) //30-35
      distance = -0.25*median +74.25;
    else if (median >= 144) //35-40 //35 onwards is bad
      distance = 34.3551+9.2450/(median-141.46911);
    else if (median >= 105) //40-50
      distance = (-0.32063)*(median) + 83.7398;
    else 
      distance = 51.0;
    return distance - D1_OFFSET;

  case D2:
      // check out of range
      if(median > 586)
        distance = 7.0;
      else if(median < 109)
        distance = 51.0;
      else
        distance = -0.87161 + 5127.804/(median-8.158281);
      return distance - D2_OFFSET;

  case D3:
      // check out of range
      if (median > 635)
        distance = 7.0;
      else if (median >= 591) // 7-8cm
        distance = 21.43182 - 0.02273*median;
      else if (median >= 168) // 8-35cm
        distance = -1.28379 + 5321.748/(median-21.49596);
      else if (median >= 142) // 35-40cm
        distance = 61.20559 + 2894.61/(median-278.5809);
      else if (median >= 114) // 40-50cm
        distance = 90.81596 - 0.31157*median;
      else
        distance = 51.0; // median < 114, out of range
      return distance - D3_OFFSET;

  case S1:  // GO
      // check out of range
      if (median > 622)
        distance = 7.0;
      else if (median >= 125) // 7-40cm
        distance = -1.82125 + 5790.725/(median+12.6569);
      else if (median >= 101) // 40-47.5cm
        distance = 79.0625 - 0.3125*median;
      else
        distance = 48.0; // median < 101, out of range
      return distance - S1_OFFSET;

  case S2:  // 30cm=29.7cm, 30-35+cm is bad, 40cm=38~39cm
      // check out of range
      if (median > 639)
        distance = 7.0;
      else if (median >= 539) // 7-9cm
        distance = -0.02*median + 19.78;
      else if (median >= 194) // 9-28cm
        distance = -2.05173 + 6108.373/(median+9.203);
//      else if (median >= 149) // 28-35cm
//        distance = 13.7867 + 1962.2/(median-56.3288);
      else if (median >= 141) // 28-38cm
        distance = 19.8196 + 818.444/(median-95.765);
      else if (median >= 105) // 35-50cm
        distance = median*median*(0.000684) - 0.5175*median + 97.0153;
      else
        distance = 51.0; // median < 105, out of range
      return distance - S2_OFFSET;
      
  case LR:
      if(median > 550)
        distance = 13.0; // out of range
      else if (median >= 539) // 13-15cm
        distance = 123 - 0.2*median;
      else if (median >= 499) // 15-20cm
        distance = 65.7006 + 18567.84/(median-905.1389);
      else if (median >= 398)  // 20-30cm
        distance = 69.42099 - 0.09901*median;
      else if (median >= 353) // 30-35cm
        distance = 76.17186 - 0.1162*median;
      else if (median >= 160) // 35-40cm
        distance = -0.12527 + 12186.82/(median-7.023576);
      else if (median >= 276) // 40-44cm
        distance = 0.003888*median*median -2.40388*median + 411.2333;
      else if (median >= 228) // 44-54cm
        distance = 0.0019645*median*median -1.19897*median + 225.26526;
      else if (median >= 204) // 54-60cm
        distance = 0.0038898*median*median -1.92879*median + 291.5673;
      else if (median >= 184) // 60-66cm
        distance = -0.006955*median*median +2.39829*median -139.7925;
      else if (median >= 169) // 66-72cm
        distance = 0.0041723*median*median -1.87928*median + 270.48473;
      else if (median >= 152) // 72-80cm
        distance = 0.013080*median*median -4.61753*median + 479.73486;
      else
        distance = 80.0;  // median < 160, out of range
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
