#include "main.h"
#include "robot.hpp"

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  okapi::Controller controller;
  auto buttonA = controller[okapi::ControllerDigital::A];

  while(true){
    controller.setText(0, 0, "A");

    while(!buttonA.changedToPressed()){

      robot::chassis->set({
        140 * controller.getAnalog(okapi::ControllerAnalog::leftY),
       -140 * controller.getAnalog(okapi::ControllerAnalog::leftX),
         -5 * controller.getAnalog(okapi::ControllerAnalog::rightX)
      });

      pros::delay(10);

    }

    controller.setText(0, 0, "B");

    while(!buttonA.changedToPressed()){

      robot::slewChassis->set({
        controller.getDigital(okapi::ControllerDigital::R1) ? 140 : 0,
        std::atan2(-controller.getAnalog(okapi::ControllerAnalog::leftX), controller.getAnalog(okapi::ControllerAnalog::leftY)),
        -5 * controller.getAnalog(okapi::ControllerAnalog::rightX)
      });

      pros::delay(10);

    }
  }
}
