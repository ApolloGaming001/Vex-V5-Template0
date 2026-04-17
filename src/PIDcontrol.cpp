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

double clamps(double val, double minVal, double maxVal) {
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

    inte = clamps(inte, -50, 50);

    double pwr = (fwd_p * err) + (fwd_d * deriv) + (fwd_i * inte);

    pwr = clamps(pwr, -12, 12);
    
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

    inte = clamps(inte, -50, 50);

    double pwr = (trn_p*err) + (trn_i*inte) + (trn_d*deriv);

    pwr = clamps(pwr,-12,12);

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

    inte = clamps(inte, -50, 50);

    double pwr = (fwd_p * err) + (fwd_i * inte) + (fwd_d * deriv);

    pwr = clamps(pwr, -12, 12);

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

    inte = clamps(inte, -50, 50);
    
    double pwr = (err*trn_p) + (inte*trn_i) + (deriv*trn_d);

    pwr = clamps(pwr, -12, 12);

    Right.spin(forward, pwr, volt);
    Left.spin(reverse, pwr, volt);

    if (fabs(err) < 3) break;

    last_err = err;

    wait(10,msec);
  }
  Drive.stop(hold);
}

void go_to(double x, double y)
{
 double rx = x - cords.at(0);
 double ry = y - cords.at(1);
 double r_mag = sqrt(pow(rx,2) + pow(ry,2));
 std::vector<double> dcords = {rx,ry};

 double lo_dist = 1e9;
 double th = 0.7;
 std::vector<double> closest;
 
 double angle;

 for (std::vector<double> obj : objects)
 {
  if (obj == dcords) continue;
  double rox = obj.at(0) - cords.at(0);
  double roy = obj.at(1) - cords.at(1);

  double o_mag = sqrt(pow(rox,2) + pow(roy,2));
  double dot = (rx*rox) + (ry*roy);
  double proj = dot / o_mag;
  
  if (proj < th || o_mag > r_mag || lo_dist > o_mag) continue;
  
  lo_dist = o_mag;
  closest = obj;
 }

 if (closest.empty())
 {
  angle = atan2(rx,ry);
  while (angle > 180)  angle -= 360;
  while (angle < -180)  angle += 360;

  d_rgt(angle);
  d_forwd(r_mag);
 } else
 {
  std::vector<double> new_pos;
  bool clear = 0;
  double dist = 0;
  bool c = 0;
  while (!clear)
  {
    dist ++;
    c=0;
    for (int i=0;i<4;i++)
    {
      if (clear) break;
      if (i == 2) c=1;
      new_pos = closest;

      if (!c)
      {
        if (i==0)
        {
          new_pos.at(0) = closest.at(0)+dist;
        }

        if (i==1)
        {
          new_pos.at(0) = closest.at(0)-dist;
        }
      }

      if (c)
      {
        if (i==2)
        {
          new_pos.at(1) = closest.at(1)+dist;
        }
        
        if (i==3)
        {
          new_pos.at(1) = closest.at(1)-dist;
        }
      }
      for (std::vector<double> obj:objects)
      {
        if (new_pos == obj) continue;
        if (new_pos == cords) continue;
        if (!(new_pos == obj)) clear = 1; break;
      }
    }

    double rnx = closest.at(0) - cords.at(0);
    double rny = closest.at(1) - cords.at(1);
    double rn_mag = sqrt(pow(rnx,2) + pow(rny,2));

    angle = atan2(rnx,rny);
    d_forwd(rn_mag);
    go_to(x,y);
  }
 }
}