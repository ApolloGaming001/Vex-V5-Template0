#include "PIDcontrol.h"
#include <vector>
#include <algorithm>

double full = 360;
double tile = 24;
double wc = 8.64;
//bot variables
double fwd_p = 0.2;
double fwd_i = 0.001;
double fwd_d = 0.35;
//forward PID variables
double trn_p = 0.4;
double trn_i = 0.003;
double trn_d = 0.4;
//turn PID variables
double dt = 0.01;
//delta time for more accurate PID
double min_xy = 0;
double max_y = 6;
double max_x = 3;
//coordinate min and max
std::vector<double> cords = {3,1};
//start coordinates (base is top of parking zone (3,1))
std::vector<std::vector<double>> objects = {
  {1,2},
  {5,2},
  {2.5,3},
  {3.5,3}
};
//PushBack object cordinates
double clamps(double val, double minVal, double maxVal) {
  return fmax(minVal, fmin(val, maxVal));
}
//vex firmware doesnt support clamp but it wont compile if I name it clamp because my compiler has clamp already
void forwd(double multi)
{
  double err = 0, inte = 0, deriv = 0, last_err = 0;

  double target = (tile * multi) + (Drive.position(deg) / 360.0 * wc);
  //define error, integral, derivitave, last error, and the target position
  while (1)
  {
    err = target - (Drive.position(deg) / 360.0 * wc);
    inte += err * dt;
    deriv = (err - last_err) / dt;
    //set the error, integral, and derivative
    inte = clamps(inte, -50, 50);
    //clamp the integral so it doesn't go waaay over
    double pwr = (fwd_p * err) + (fwd_d * deriv) + (fwd_i * inte);
    //define and calculate the power
    Drive.spin(forward,pwr,volt);
    //spin the Drivetrain forward at the power using volts as the unit
    if (fabs(err) < 3) // if the error is less that 3
    {
      break;
    }

    last_err = err;
    //set the last error to the current error
    wait(10,msec);
    //give CPU time to process
  }
  Drive.stop(hold);
}

void rever(double multi)
{
  forwd(-multi);
}
//reverse PID
void rgt(double multi)
{
  double err = 0, inte = 0, deriv = 0, last_err = 0;
  //defining error, integral, derivative and last error
  double start = Gyroscope.rotation(deg);
  //defining the start rotation
  double target = (90*multi) + start;
  //defining the target rotation
  while (1)
  {
    double curr = Gyroscope.rotation(deg);
    //establish the current rotation
    err = target - curr;
    inte += err*dt;
    deriv = (err-last_err)/dt;
    //set the current error, integral, and derivative
    inte = clamps(inte, -50, 50);
    //clamp the integral
    double pwr = (trn_p*err) + (trn_i*inte) + (trn_d*deriv);
    //define the power

    Right.spin(reverse,pwr,volt);
    Left.spin(forward,pwr,volt);
    //spin the bot with the power using volage as the unit of power
    last_err = err;
    //set the last error to the current error
    if (fabs(err) < 3) break;
    //if the absolute value of the error is less than 3 then break
    wait(10,msec);
    //give CPU time to porcess
  }
  Drive.stop(hold);
  //brake
}

void lft(double multi)
{
  rgt(-multi);
}

void d_forwd(double dist)
{
  //same thing as the regular forward except it uses the coordinates
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
//reverse coordinate PID
void d_rgt(double ang)
{
  //same as regular rgt except it uses coordinates and degrees
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

    Right.spin(reverse, pwr, volt);
    Left.spin(forward, pwr, volt);

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
 //define the relative x, y, magnitude, and the desired cords vector
 double lo_dist = 1e9;
 double th = 0.7;
 std::vector<double> closest;
 //define the last object distance, threshhold for the dot projection, and the closest object's vector
 double angle;
 //define the angle variable
 for (std::vector<double> obj : objects)
 //loop through the objects
 {
  if (obj == dcords) continue; //if the object coords are the desired coords
  double rox = obj.at(0) - cords.at(0);
  double roy = obj.at(1) - cords.at(1);
  double o_mag = sqrt(pow(rox,2) + pow(roy,2));
  //define the relative object x, y, and magnitude
  double dot = (rx*rox) + (ry*roy);
  double proj = dot / o_mag;
  //define the dot and its projection onto the desired coords vector
  if (proj < th || o_mag > r_mag || lo_dist < o_mag) continue;
  //if the projection is greator than the threshold or the object's magnitude is less than the desired magnitude or the last object
  //'s magnitude is less than the object's magnitude then continue to the next object
  lo_dist = o_mag;
  closest = obj;
  //else the last object's magnitude is equal to the object's magnitude and the closest object is the object
 }

 if (closest.empty())
 //if there is no object in the way
 {
  angle = atan2(rx,ry);
  while (angle > 180)  angle -= 360;
  while (angle < -180)  angle += 360;
  //set the angle to the arc tan of the relative x and y and normalize it
  d_rgt(angle);
  d_forwd(r_mag);
  //turn to the angle using d_rgt and move the bot the relative magnitude
  cords = {x,y};
 } else
 {//if there is an object in the way
  std::vector<double> new_pos;
  bool clear = 0;
  double dist = 0;
  bool c = 0;
  //define a new set of coords, make a flag for the while loop, set a distance is coord tiles from the current object, and a check
  //for the current value to switch
  while (!clear)
  {//while the flag is false
    dist ++;
    c=0;
    //add to the dist and reset the value switch
    for (int i=0;i<4;i++)
    {//run 4 times
      if (clear) break;// if the flag is true then break
      if (i == 2) c=1; //when the i = 2 switch the value that changes
      new_pos = closest;
      //reset the new coords to the current object
      if (!c)
      {//first, run the x value
        if (i==0)
        {//on the first run
          new_pos.at(0) = closest.at(0)+dist;//add to the x
        }

        if (i==1)
        {//on the second run
          new_pos.at(0) = closest.at(0)-dist;//subtract from the x
        }
      }

      if (c)
      {//second, run the y value if the x value isn't clear
        if (i==2)
        {//on the third run
          new_pos.at(1) = closest.at(1)+dist;//add to the y
        }
        
        if (i==3)
        {//on the fourth run
          new_pos.at(1) = closest.at(1)-dist;//subract from the y
        }
      }
      for (std::vector<double> obj:objects)
      {//loop through all objects
        if (new_pos == obj) continue;
        if (new_pos == cords) continue;
        //if the new position is either equal to the current coordinate or if its equal to the objects position then continue
        double rox = obj.at(0) - cords.at(0);
        double roy = obj.at(1) - cords.at(1);
        double ro_mag = sqrt(pow(rox,2) + pow(roy,2));
        //define the relative object x,y and magnitude
        double rnx = new_pos.at(0) - cords.at(0);
        double rny = new_pos.at(1) - cords.at(1);
        //define the relative new x, and y
        double dot = ((rox*rnx) + (roy*rny)) / ro_mag;
        //define the dot
        if (dot < th) continue; //if the dot is less than the threshold then continue
        clear = 1; //else clear is true
        break; //break the loop
      }
    }
  }
  double rnx = new_pos.at(0) - cords.at(0);
  double rny = new_pos.at(1) - cords.at(1);
  double rn_mag = sqrt(pow(rnx,2) + pow(rny,2));
  angle = atan2(rnx,rny);
  //define the relative new x, y, magnitude, and the turn angle
  d_rgt(angle);
  d_forwd(rn_mag);
  //turn to the angle and move the new pos magnitude
  cords = new_pos;
  go_to(x,y);
  //update cords and call go_to() again using the same x,y to finally get to the destination
 }
}
//ong ts should work but if it doesn't ima eat all of my friend's fried chicken from popeyes.