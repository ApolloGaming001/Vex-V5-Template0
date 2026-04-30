#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

controller C = controller(primary);
motor r1 = motor(PORT11, ratio6_1, 1);
motor r2 = motor(PORT12, ratio6_1, 1);
motor_group Right = motor_group(r1,r2);
motor l1 = motor(PORT18, ratio6_1);
motor l2 = motor(PORT19, ratio6_1);
motor_group Left = motor_group(l1,l2);
motor_group Drive = motor_group(r1,r2,l1,l2);
inertial Gyroscope = inertial(PORT21);

void vexcodeInit(void) {
  // Nothing to initialize
}