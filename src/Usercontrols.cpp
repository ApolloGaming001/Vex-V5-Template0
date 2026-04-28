#include "Usercontrols.h"

double clamp(double val, double minVal, double maxVal) {
  return fmax(minVal, fmin(val, maxVal));
}

void main_usercontrol()
{
  float fwd = C.Axis3.position(pct);
  float trn = C.Axis1.position(pct);

  Right.spin(forward,fwd-trn,pct);
  Left.spin(forward,fwd+trn,pct);
}