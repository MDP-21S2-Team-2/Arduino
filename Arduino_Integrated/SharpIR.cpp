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
//    else if (median >= 180) //20-30
//      distance = (0.11937)+(4929.93)/(median-15.281);
////    else if (median >= 137) //33-40  bad readings off by 1cm
////      distance = 94.9596+(-0.516238)*(median) + (0.000854)*(median)*(median);
//    else if (median >= 153) //30-35
//      distance = -0.25*median +74.25;
    else if (median >= 157) // 20-35
      distance = 0.9702 + 4621.915/(median-20.9586);
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
        distance = 57.5 + 2047.5/(median-259);
      else if (median >= 114) // 40-50cm
        distance = 90.81596 - 0.31157*median;
      else
        distance = 51.0; // median < 114, out of range
      return distance - D3_OFFSET;

  case S1:  // GO
      // check out of range
      if (median > 622)
        distance = 7.0;
      else if (median>= 254) // 7 - 20 
          distance = -3.0558688 + 6792.5041/(median+41.38636);
      else if (median >= 185) // 20-27.5
          distance = 5.9462277 + 2785.5943/(median - 55.769522);
      else if (median >= 125)
          distance = 0.5984254 + 4970.4133/median;
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
      if(median > 540)
        distance = 13.0; // out of range
      else if (median >= 386) // 13-30cm
        distance = -0.00013*median*median + 0.01436*median + 44.37745;
      else if (median >= 283) // 30-43cm
        distance = -5.1981 + 13634.87/median;
      else if (median >= 245) // 43-50cm
        distance = 71.6641 + 3443.622/(median-403.436);
      else if (median >= 179) // 50-70cm
        distance = 0.001863*median*median - 1.0929*median + 205.8814;
      else if (median >= 165) // 70-75cm
        distance = -0.00128*median*median + 0.0577*median + 100.751;
      else if (median >= 130) // 75-90cm
        distance = -13.361 + 17255.81/(median+29.6083);
      else
        distance = 91.0;  // median < 135, out of range
      return distance - LR_OFFSET;
  }
}

#define NUM_MEDIANS 3 // NUM_SAMPLES / 5
#define HALF_NUM_MEDIANS 1  // NUM_MEDIANS / 2

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
