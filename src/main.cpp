#include "main.h"
#include "robot.hpp"

inline double deadzone(double val, double threshold){
  return std::abs(val) > threshold ? val : 0;
}

std::int32_t controllerSetText(std::uint8_t iline, std::uint8_t icol, std::string itext){
  static uint32_t t{0};

  if(pros::millis() - 100 < t){
    return 1;
  }else{
    t = pros::millis();
    return robot::controller->setText(iline, icol, itext);
  }
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  auto buttonA = (*robot::controller)[okapi::ControllerDigital::A];

  auto headingController = std::make_shared<kappa::PidController>(kappa::PidController::Gains{0.1,0,0.1,0});

  pros::Task odomTask([&]{
		auto t = pros::millis();

		while(true){
			robot::odometry->step();

			pros::Task::delay_until(&t, 10);
		}
	}, "Odom Task");

  pros::Task logTask([&]{
    while(true){
      auto pose = robot::odometry->get();
      std::cout << "(" << pose.x << ", " << pose.y << ", " << pose.theta << ")\n";
      pros::delay(100);
    }
  }, "Log Task");

  while(true){

    while(!buttonA.changedToPressed()){

      controllerSetText(0, 0, "A       ");

      if(robot::controller->getDigital(okapi::ControllerDigital::L1)){
        robot::intake->runAll();
      }else{

        if(robot::controller->getDigital(okapi::ControllerDigital::R1)){
          robot::intake->intake();
        }else{

          if(robot::controller->getDigital(okapi::ControllerDigital::L2)){
            robot::intake->outtake();
          }else{

            if(robot::controller->getDigital(okapi::ControllerDigital::R2)){
              robot::intake->dump();
            }else{
              robot::intake->idle();
            }
          }
        }
      }
      robot::chassis->set({
        robot::maxLinearSpeed  * robot::controller->getAnalog(okapi::ControllerAnalog::leftY),
       -robot::maxLinearSpeed  * robot::controller->getAnalog(okapi::ControllerAnalog::leftX),
       -robot::maxAngularSpeed * robot::controller->getAnalog(okapi::ControllerAnalog::rightX)
      });

      pros::delay(10);

    }

    while(!buttonA.changedToPressed()){

      controllerSetText(0, 0, "B" + std::to_string(headingController->getTarget()) + "   ");

      headingController->setTarget(headingController->getTarget() - robot::maxAngularSpeed * robot::controller->getAnalog(okapi::ControllerAnalog::rightX) * 0.01 * 180 / M_PI);

      robot::chassis->set({
        robot::maxLinearSpeed  * robot::controller->getAnalog(okapi::ControllerAnalog::leftY),
       -robot::maxLinearSpeed  * robot::controller->getAnalog(okapi::ControllerAnalog::leftX),
        deadzone(headingController->step(robot::imu->get()), 0.1)
      });

      pros::delay(10);

    }

    while(!buttonA.changedToPressed()){

      controllerSetText(0, 0, "C" + std::to_string(headingController->getTarget()) + "   ");

      headingController->setTarget(headingController->getTarget() - robot::maxAngularSpeed * robot::controller->getAnalog(okapi::ControllerAnalog::rightX) * 0.01 * 180 / M_PI);

      robot::slewChassis->set({
        robot::controller->getDigital(okapi::ControllerDigital::R1) ? robot::maxLinearSpeed : 0,
        std::atan2(-robot::controller->getAnalog(okapi::ControllerAnalog::leftX), robot::controller->getAnalog(okapi::ControllerAnalog::leftY)),
        deadzone(headingController->step(robot::imu->get()), 0.1)
      });

      pros::delay(10);

    }


    while(!buttonA.changedToPressed()){

      controllerSetText(0, 0, "D" + std::to_string(headingController->getTarget()) + "   ");

      headingController->setTarget(headingController->getTarget() - robot::maxAngularSpeed * robot::controller->getAnalog(okapi::ControllerAnalog::rightX) * 0.01 * 180 / M_PI);

      robot::slewChassis->set({
        robot::controller->getDigital(okapi::ControllerDigital::R1) ? robot::maxLinearSpeed : 0,
        std::atan2(-robot::controller->getAnalog(okapi::ControllerAnalog::leftX), robot::controller->getAnalog(okapi::ControllerAnalog::leftY)) - M_PI / 180 * robot::imu->get(),
        deadzone(headingController->step(robot::imu->get()), 0.1)
      });

      pros::delay(10);

    }

    while(!robot::poseController->isSettled()){

      controllerSetText(0, 0, "E         ");

      robot::slewChassis->set(robot::poseController->step(robot::odometry->get()));

      pros::delay(10);
    }
  }
}
