#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
  // time
  static unsigned long _targetTime;
  
  // for calculating errors
  double _lastValue;
  double _sumError;
  const double _Imax;
  const double _Imin;
  
  // PID constants
  const double _Kp;
  const double _Ki;
  const double _Kd;

  public:
    // constructor
    PID(double P, double I, double D, double Imax, double Imin/*, int minOut, int maxOut*/) :
      _Kp(P), _Ki(I), _Kd(D),
      _Imax(Imax), _Imin(Imin),
      _lastValue(0.0), _sumError(0.0) {}

      double getLastValue() { return _lastValue; }
      double getSumError() { return _sumError; }

    // compute PID function
    double computePID(double input, double setPoint);
    void resetPID();

    // check elapsed time
    static bool checkPIDCompute();
};

#endif
