#include "vex.h"
#include "Autons.h"
#include "Usercontrols.h"
using namespace vex;
competition Competition;

int switch_int = 1;

void pre_auton(void) {
  vexcodeInit();
  Gyroscope.calibrate();
  while (Gyroscope.isCalibrating()) wait(10,msec);
}

void autonomous(void) {
 switch(switch_int)
  {
    case 1:
      break;
    case 2:
      break;
    default:
      printf("ERR: NO AUTON LOADED");
  }
}

void usercontrol(void) {
  while (1) {
    main_usercontrol();
    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
