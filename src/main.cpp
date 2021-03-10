#include "main.h"
#include "robot.hpp"

inline double deadzone(double val, double threshold){
  return std::abs(val) > threshold ? val : 0;
}

std::int32_t controllerSetText(uint32_t *t, std::uint8_t iline, std::uint8_t icol, std::string itext){
  if(pros::millis() - 50 < *t){
    return 1;
  }else{
    *t = pros::millis();
    return robot::controller->setText(iline, icol, itext);
  }
}

void disabled() {}

void competition_initialize() {}

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

      //auto data = robot::sensorArray->get();
      //std::cout << data[0] << ", " << data[0] << ", " << data[0] << ", " << data[0] << "\n";
      pros::delay(100);
    }
  }, "Log Task");

  uint32_t t = pros::millis();

  while(true){

    while(!buttonA.changedToPressed()){

      controllerSetText(&t, 0, 0, "STD       ");

      if(robot::controller->getDigital(okapi::ControllerDigital::L1)){
        robot::intake->runAll();
      }else{

        if(robot::controller->getDigital(okapi::ControllerDigital::R1)){
          robot::intake->intake();
        }else{

          if(robot::controller->getDigital(okapi::ControllerDigital::L2)){
            robot::intake->outtake();
          }else{

            if(robot::controller->getDigital(okapi::ControllerDigital::Y)){
              robot::intake->dump();
            }else{

              if(robot::controller->getDigital(okapi::ControllerDigital::right)){
                robot::intake->runBField(0b00000101);
              }else{
                robot::intake->idle();
              }
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

      controllerSetText(&t, 0, 0, "Smrt" + std::to_string(headingController->getTarget()) + "   ");

      if(robot::controller->getDigital(okapi::ControllerDigital::L1)){
        robot::intake->runAll();
      }else{

        if(robot::controller->getDigital(okapi::ControllerDigital::R1)){
          robot::intake->intake();
        }else{

          if(robot::controller->getDigital(okapi::ControllerDigital::L2)){
            robot::intake->outtake();
          }else{

            if(robot::controller->getDigital(okapi::ControllerDigital::Y)){
              robot::intake->dump();
            }else{

              if(robot::controller->getDigital(okapi::ControllerDigital::right)){
                robot::intake->runBField(0b00000101);
              }else{
                robot::intake->idle();
              }
            }
          }
        }
      }

      headingController->setTarget(headingController->getTarget() - robot::maxAngularSpeed * deadzone(robot::controller->getAnalog(okapi::ControllerAnalog::rightX), 0.1) * 0.01 * 180 / M_PI);

      robot::chassis->set({
        robot::maxLinearSpeed  * robot::controller->getAnalog(okapi::ControllerAnalog::leftY),
       -robot::maxLinearSpeed  * robot::controller->getAnalog(okapi::ControllerAnalog::leftX),
        deadzone(headingController->step(robot::imu->get()), 0.1)
      });

      pros::delay(10);

    }

    while(!buttonA.changedToPressed()){

      controllerSetText(&t, 0, 0, "FCentric" + std::to_string(headingController->getTarget()) + "   ");

      if(robot::controller->getDigital(okapi::ControllerDigital::L1)){
        robot::intake->runAll();
      }else{

        if(robot::controller->getDigital(okapi::ControllerDigital::R1)){
          robot::intake->runBField(0b11111100);
        }else{

          if(robot::controller->getDigital(okapi::ControllerDigital::R2)){
            robot::intake->intake();
          }else{

            if(robot::controller->getDigital(okapi::ControllerDigital::L2)){
              robot::intake->outtake();
            }else{

              if(robot::controller->getDigital(okapi::ControllerDigital::Y)){
                robot::intake->dump();
              }else{

                if(robot::controller->getDigital(okapi::ControllerDigital::right)){
                  robot::intake->runBField(0b00000101);
                }else{
                  robot::intake->idle();
                }
              }
            }
          }
        }
      }

      headingController->setTarget(headingController->getTarget() - robot::maxAngularSpeed * deadzone(robot::controller->getAnalog(okapi::ControllerAnalog::rightX), 0.1) * 0.01 * 180 / M_PI);

      robot::slewChassis->set({
        robot::maxLinearSpeed * sqrt(pow(robot::controller->getAnalog(okapi::ControllerAnalog::leftX), 2) + pow(robot::controller->getAnalog(okapi::ControllerAnalog::leftY), 2)),
        std::atan2(-robot::controller->getAnalog(okapi::ControllerAnalog::leftX), robot::controller->getAnalog(okapi::ControllerAnalog::leftY)) - M_PI / 180 * robot::imu->get(),
        deadzone(headingController->step(robot::imu->get()), 0.1)
      });

      pros::delay(10);

    }

    while(!robot::poseController->isSettled()){

      controllerSetText(&t, 0, 0, "Auto       ");

      robot::slewChassis->set(robot::poseController->step(robot::odometry->get()));

      pros::delay(10);
    }
  }
}
