#include "PID.h"

#define LOOPTIME 20000

unsigned long PID::_targetTime = 0;

double PID::computePID(double input, double setPoint) {
  
  double pterm;
  double iterm;
  double dterm;
  double error;
  
  // compute the error
  error = setPoint - input;
  
  // proportional term
  pterm = _Kp * error;
  
  // integral term
  _sumError += error; //* dt;
  // clamp the integral term
  if (_sumError > _Imax) {
    _sumError = _Imax;
  } else if (_sumError < _Imin) {
    _sumError = _Imin;
  }
  iterm = _Ki * _sumError;
  
  // derivative term
  dterm = _Kd * (_lastValue - input);  // last RPM - current RPM
  _lastValue = input;

  return pterm + iterm + dterm;
}

void PID::resetPID() {
  _sumError = 0;
  _lastValue = 0;
}

bool PID::checkPIDCompute() {
  unsigned long elapsedTime = micros();
  if (elapsedTime > _targetTime) {
    _targetTime = elapsedTime + LOOPTIME;
    return true;
  }
  return false;
}
