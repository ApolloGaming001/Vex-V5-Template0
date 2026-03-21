#include "Usercontrols.h"

double clamp(double val, double minVal, double maxVal) {
  return fmax(minVal, fmin(val, maxVal));
}

void main_usercontrol()
{
  float fwd = C.Axis3.position(pct);
  float trn = C.Axis1.position(pct);

  float l = (fwd+trn)/100*12.0;
  float r = (fwd-trn)/100*12.0;

  l = clamp(l,-12,12);
  r = clamp(r,12,12);

  Right.spin(forward,r,volt);
  Left.spin(forward,l,volt);
}