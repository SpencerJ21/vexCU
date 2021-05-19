#include "main.h"
#include "robot.hpp"

void disabled() {}

void competition_initialize() {}

std::int32_t controllerSetText(uint32_t *t, std::uint8_t iline, std::uint8_t icol, std::string itext){
  if(pros::millis() - 50 < *t){
    return 1;
  }else{
    *t = pros::millis();
    return robot::controller->setText(iline, icol, itext);
  }
}

void opcontrol() {
  pros::Task logTask([&]{
    while(true){
      auto pose = robot::odometry->get();
      std::cout << "(" << pose.x << ", " << pose.y << ", " << pose.theta << ")\n";

      //auto data = robot::sensorArray->get();
      //std::cout << data[0] << ", " << data[0] << ", " << data[0] << ", " << data[0] << "\n";
      pros::delay(100);
    }
  }, "Log Task");

  uint32_t startTime = pros::millis();

  uint32_t t = startTime;

  while(true){

    controllerSetText(&t, 0, 0, "STD       ");

    if(robot::controller->getDigital(okapi::ControllerDigital::up) || startTime + 500 > pros::millis()){
      robot::intake->execute(-8000, 0, 0);

    }else if(robot::controller->getDigital(okapi::ControllerDigital::L1)){
      robot::intake->execute(12000,12000,12000);

    }else if(robot::controller->getDigital(okapi::ControllerDigital::R1)){
      robot::intake->execute(12000,12000,0);

    }else if(robot::controller->getDigital(okapi::ControllerDigital::L2)){
      robot::intake->execute(0,12000,12000);

    }else if(robot::controller->getDigital(okapi::ControllerDigital::Y)){
      robot::intake->execute(-12000,-12000,-12000);

    }else if(robot::controller->getDigital(okapi::ControllerDigital::right)){
      robot::intake->execute(0,-12000,-12000);

    }else{
      robot::intake->execute(0,0,0);

    }

    double driveScalar = robot::controller->getDigital(okapi::ControllerDigital::R2) ? 0.5 : 1.0;

    robot::chassis->set({
      driveScalar * robot::maxLinearSpeed  * robot::controller->getAnalog(okapi::ControllerAnalog::leftY),
     -driveScalar * robot::maxLinearSpeed  * robot::controller->getAnalog(okapi::ControllerAnalog::leftX),
     -driveScalar * robot::maxAngularSpeed * robot::controller->getAnalog(okapi::ControllerAnalog::rightX)
    });

    pros::delay(10);
  }
}
