#include "main.h"
#include "robot.hpp"

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  okapi::Controller controller;
  auto buttonA = controller[okapi::ControllerDigital::A];

  while(true){
    while(!buttonA.changedToPressed()){

      robot::chassis->set({
        140 * controller.getAnalog(okapi::ControllerAnalog::leftY),
       -140 * controller.getAnalog(okapi::ControllerAnalog::leftX),
         -5 * controller.getAnalog(okapi::ControllerAnalog::rightX)
      });

      pros::delay(10);

    }

    while(!buttonA.changedToPressed()){

      robot::chassis->setPolar({
        140 * controller.getAnalog(okapi::ControllerAnalog::rightY),
        std::atan2(-controller.getAnalog(okapi::ControllerAnalog::leftX), controller.getAnalog(okapi::ControllerAnalog::leftY)),
        0.0
      });

      pros::delay(10);

    }
  }
}
