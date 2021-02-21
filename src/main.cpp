#include "main.h"
#include "robot.hpp"

inline double deadzone(double val, double threshold){
  return std::abs(val) > threshold ? val : 0;
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  okapi::Controller controller;
  auto buttonA = controller[okapi::ControllerDigital::A];

  auto headingController = std::make_shared<kappa::PidController>(kappa::PidController::Gains{0.1,0,0.1,0});

  while(true){

    while(!buttonA.changedToPressed()){

      controller.setText(0, 0, "A       ");

      robot::chassis->set({
        robot::maxLinearSpeed  * controller.getAnalog(okapi::ControllerAnalog::leftY),
       -robot::maxLinearSpeed  * controller.getAnalog(okapi::ControllerAnalog::leftX),
       -robot::maxAngularSpeed * controller.getAnalog(okapi::ControllerAnalog::rightX)
      });

      pros::delay(10);

    }

    while(!buttonA.changedToPressed()){

      controller.setText(0, 0, "B" + std::to_string(headingController->getTarget()) + "   ");

      headingController->setTarget(headingController->getTarget() - robot::maxAngularSpeed * controller.getAnalog(okapi::ControllerAnalog::rightX) * 0.01 * 180 / M_PI);

      robot::chassis->set({
        robot::maxLinearSpeed  * controller.getAnalog(okapi::ControllerAnalog::leftY),
       -robot::maxLinearSpeed  * controller.getAnalog(okapi::ControllerAnalog::leftX),
        deadzone(headingController->step(robot::imu->get()), 0.1)
      });

      pros::delay(10);

    }

    while(!buttonA.changedToPressed()){

      controller.setText(0, 0, "C" + std::to_string(headingController->getTarget()) + "   ");

      headingController->setTarget(headingController->getTarget() - robot::maxAngularSpeed * controller.getAnalog(okapi::ControllerAnalog::rightX) * 0.01 * 180 / M_PI);

      robot::slewChassis->set({
        controller.getDigital(okapi::ControllerDigital::R1) ? robot::maxLinearSpeed : 0,
        std::atan2(-controller.getAnalog(okapi::ControllerAnalog::leftX), controller.getAnalog(okapi::ControllerAnalog::leftY)),
        deadzone(headingController->step(robot::imu->get()), 0.1)
      });

      pros::delay(10);

    }


    while(!buttonA.changedToPressed()){

      controller.setText(0, 0, "D" + std::to_string(headingController->getTarget()) + "   ");

      headingController->setTarget(headingController->getTarget() - robot::maxAngularSpeed * controller.getAnalog(okapi::ControllerAnalog::rightX) * 0.01 * 180 / M_PI);

      robot::slewChassis->set({
        controller.getDigital(okapi::ControllerDigital::R1) ? robot::maxLinearSpeed : 0,
        std::atan2(-controller.getAnalog(okapi::ControllerAnalog::leftX), controller.getAnalog(okapi::ControllerAnalog::leftY)) - M_PI / 180 * robot::imu->get(),
        deadzone(headingController->step(robot::imu->get()), 0.1)
      });

      pros::delay(10);

    }
  }
}
