#include "main.h"
#include "robot.hpp"

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  okapi::Controller controller;
  auto buttonA = controller[okapi::ControllerDigital::A];

  auto headingController = std::make_shared<kappa::PidController>(kappa::PidController::Gains{4,0,0,0});

  while(true){
    controller.setText(0, 0, "A");

    while(!buttonA.changedToPressed()){

      robot::chassis->set({
        robot::maxLinearSpeed  * controller.getAnalog(okapi::ControllerAnalog::leftY),
       -robot::maxLinearSpeed  * controller.getAnalog(okapi::ControllerAnalog::leftX),
       -robot::maxAngularSpeed * controller.getAnalog(okapi::ControllerAnalog::rightX)
      });

      pros::delay(10);

    }

    controller.setText(0, 0, "B" + std::to_string(headingController->getTarget() * 180 / M_PI));

    while(!buttonA.changedToPressed()){

      headingController->setTarget(headingController->getTarget() - robot::maxAngularSpeed * controller.getAnalog(okapi::ControllerAnalog::rightX) * 0.01);

      robot::slewChassis->set({
        controller.getDigital(okapi::ControllerDigital::R1) ? robot::maxLinearSpeed :
            controller.getDigital(okapi::ControllerDigital::R2) ? -robot::maxLinearSpeed : 0,
        std::atan2(-controller.getAnalog(okapi::ControllerAnalog::leftX), controller.getAnalog(okapi::ControllerAnalog::leftY)),
        headingController->step(robot::imu->get())
      });

      pros::delay(10);

    }
  }
}
