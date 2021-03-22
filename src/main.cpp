#include "main.h"
#include "robot.hpp"

void disabled() {}

void competition_initialize() {}



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

void opcontrol() {
  auto buttonA = (*robot::controller)[okapi::ControllerDigital::A];

  auto headingController = std::make_shared<kappa::PidController>(kappa::PidController::Gains{0.1,0,0.1,0});

  pros::Task logTask([&]{
    while(true){
      auto pose = robot::odometry->get();
      std::cout << "(" << pose.x << ", " << pose.y << ", " << pose.theta << ")\tS=" << robot::intake->getSensorValue() << "\n";

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
      robot::intake->runBField(0b01010000);

    }else if(robot::controller->getDigital(okapi::ControllerDigital::L1)){
      robot::intake->runAll();

    }else if(robot::controller->getDigital(okapi::ControllerDigital::R1)){
      robot::intake->intake();

    }else if(robot::controller->getDigital(okapi::ControllerDigital::L2)){
      robot::intake->outtake();

    }else if(robot::controller->getDigital(okapi::ControllerDigital::Y)){
      robot::intake->dump();

    }else if(robot::controller->getDigital(okapi::ControllerDigital::right)){
      robot::intake->runBField(0b00000101);

    }else{
      robot::intake->idle();

    }

    double driveScalar = robot::controller->getDigital(okapi::ControllerDigital::R2) ? 0.5 : 1.0;

    robot::driverChassis->set({
      driveScalar * robot::maxLinearSpeed  * robot::controller->getAnalog(okapi::ControllerAnalog::leftY),
     -driveScalar * robot::maxLinearSpeed  * robot::controller->getAnalog(okapi::ControllerAnalog::leftX),
     -driveScalar * robot::maxAngularSpeed * robot::controller->getAnalog(okapi::ControllerAnalog::rightX)
    });

    pros::delay(10);
  }
}
