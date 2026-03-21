#include "PIDcontrol.h"
#include <vector>
#include <algorithm>

double full = 360;
double tile = 24;
double wc = 8.64;

double fwd_p = 0.2;
double fwd_i = 0.001;
double fwd_d = 0.35;

double trn_p = 0.4;
double trn_i = 0.003;
double trn_d = 0.4;

double dt = 0.01;

double min_xy = 0;
double max_y = 6;
double max_x = 3;

std::vector<double> cords = {3,1};

std::vector<std::vector<double>> objects = {
  {1,2},
  {5,2},
  {2.5,3},
  {3.5,3}
};

std::vector<std::vector<double>> walls = {
  {0,3},
  {0,6}
};

double clamp(double val, double minVal, double maxVal) {
  return fmax(minVal, fmin(val, maxVal));
}

void forwd(double multi)
{
  double err = 0, inte = 0, deriv = 0, last_err = 0;

  double target = (tile * multi) + (Drive.position(deg) / 360.0 * wc);

  while (1)
  {
    err = target - (Drive.position(deg) / 360.0 * wc);
    inte += err * dt;
    deriv = (err - last_err) / dt;

    inte = clamp(inte, -50, 50);

    double pwr = (fwd_p * err) + (fwd_d * deriv) + (fwd_i * inte);

    pwr = clamp(pwr, -12, 12);
    
    Drive.spin(forward,pwr,volt);

    if (fabs(err) < 3)
    {
      break;
    }

    last_err = err;

    wait(10,msec);
  }
  Drive.stop(hold);
}

void rever(double multi)
{
  forwd(-multi);
}

void rgt(double multi)
{
  double err = 0, inte = 0, deriv = 0, last_err = 0;

  double start = Gyroscope.rotation(deg);

  double target = (90*multi) + start;

  while (1)
  {
    double curr = Gyroscope.rotation(deg);

    err = target - curr;
    inte += err*dt;
    deriv = (err-last_err)/dt;

    inte = clamp(inte, -50, 50);

    double pwr = (trn_p*err) + (trn_i*inte) + (trn_d*deriv);

    pwr = clamp(pwr,-12,12);

    Right.spin(forward,pwr,volt);
    Left.spin(reverse,pwr,volt);

    last_err = err;

    if (fabs(err) < 3) break;

    wait(10,msec);
  }
  Drive.stop(hold);
}

void lft(double multi)
{
  rgt(-multi);
}

void d_forwd(double dist)
{
  double err = 0, inte = 0, deriv = 0, last_err = 0;

  double target = (dist*tile) + (Drive.position(deg)/360*wc);

  while (1)
  {
    err = target - (Drive.position(deg)/360*wc);
    inte += err * dt;
    deriv = (err-last_err) / dt;

    inte = clamp(inte, -50, 50);

    double pwr = (fwd_p * err) + (fwd_i * inte) + (fwd_d * deriv);

    pwr = clamp(pwr, -12, 12);

    Drive.spin(forward, pwr, volt);

    if (fabs(err) < 3) break;
    
    last_err = err;

    wait(10,msec);
  }
  Drive.stop(hold);
}

void d_rever(double dist)
{
  d_forwd(-dist);
}

void d_rgt(double ang)
{
  double err = 0, inte = 0, deriv = 0, last_err = 0;
  double start = Gyroscope.rotation(deg);

  double target = start + ang;

  while (1)
  {
    double curr = Gyroscope.rotation(deg);

    err = target - curr;
    inte += err * dt;
    deriv = (err - last_err) / dt;

    inte = clamp(inte, -50, 50);
    
    double pwr = (err*trn_p) + (inte*trn_i) + (deriv*trn_d);

    pwr = clamp(pwr, -12, 12);

    Right.spin(forward, pwr, volt);
    Left.spin(reverse, pwr, volt);

    if (fabs(err) < 3) break;

    last_err = err;

    wait(10,msec);
  }
  Drive.stop(hold);
}

void d_lft(double ang)
{
  d_rgt(-ang);
}

void go_to(double x, double y)
{
 x = clamp(x, min_xy, max_x);
 y = clamp(y, min_xy, max_y);
 
 std::vector<double> target_cords = {x,y};
 double dx = target_cords.at(0) - cords.at(0);
 double dy = target_cords.at(1) - cords.at(1);
 double target_dist = sqrt(pow(dx,2) + pow(dy,2));

 double heading = Gyroscope.heading(deg);
 double angle = (atan2(dx,dy) * (180/M_PI)) - heading;

 double th = 0.95;

 std::vector<double> closest;
 double l_obj_dist = 1e9;

 for (std::vector<double> object : objects)
 {
  if (target_cords == object) continue;

  double ox = object.at(0) - cords.at(0);
  double oy = object.at(1) - cords.at(1);
  
  double o_dist = sqrt(pow(ox,2) + pow(oy,2));

  double dot = ((dx*ox) + (dy*oy)) / (target_dist * o_dist);

  if ((closest.empty() || l_obj_dist > o_dist) && dot > th)
  {
    closest = object;
    l_obj_dist = o_dist;
  }
 }

 if (!(closest.empty()))
 {
  if (closest.at(1)-1 == 0)
  {
    go_to(closest.at(0)+1, closest.at(1));
  }else
  {
    go_to(closest.at(0), closest.at(1)+1);
  }
 }

 while (angle < -180) angle += full;
 while (angle > 180) angle -= full;

 if (angle > 0) d_lft(angle);
 else d_rgt(angle);
 d_forwd(target_dist);

 cords = target_cords;
}