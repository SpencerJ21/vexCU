#include "main.h"
#include "robot.hpp"

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  okapi::Controller controller;

  while(true){
    robot::chassis->set({
      0,0,0
    });
  }
}
