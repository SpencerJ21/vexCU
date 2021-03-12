#include "main.h"
#include "robot.hpp"

inline void chassisWait(){
  do{
    robot::slewChassis->set(robot::poseController->step(robot::odometry->get()));
    pros::delay(10);
  }while(!robot::poseController->isSettled());
}


void autonomous() {

  robot::poseController->setTarget({4, 0, 0, 0, 0, 0});

  robot::intake->runBField(0b00001000);
  pros::delay(500);

  robot::intake->intake();

  robot::poseController->setStopping(false);
  robot::poseController->setTarget({30, 5, M_PI_2, 0, 0, 0});
  chassisWait();

  robot::poseController->setStopping(true);
  robot::poseController->setTarget({30, 14, M_PI_2, 0, 0, 0});
  chassisWait();
  pros::delay(1000);

  robot::intake->idle();
  robot::poseController->setStopping(false);
  robot::poseController->setTarget({22, 12, M_PI_2 + M_PI_4, 0, 0, 0});
  chassisWait();

  robot::poseController->setStopping(true);
  robot::poseController->setTarget({18, 15, M_PI_2 + M_PI_4, 0, 0, 0});
  chassisWait();

  robot::intake->runAll();
  pros::delay(1500);

  robot::intake->dump();
  robot::poseController->setStopping(false);
  robot::poseController->setTarget({36, -12, M_PI_2, 0, 0, 0});
  chassisWait();

  robot::intake->intake();
  robot::poseController->setStopping(true);
  robot::poseController->setTarget({54, -12, 0, 0, 0, 0});
  chassisWait();

  robot::poseController->setStopping(false);
  robot::poseController->setTarget({70, -6, M_PI_2, 0, 0, 0});
  chassisWait();

  robot::poseController->setStopping(true);
  robot::poseController->setTarget({70, 12, M_PI_2, 0, 0, 0});
  chassisWait();

  robot::intake->runAll();
  pros::delay(1500);

  robot::intake->dump();
  robot::poseController->setStopping(true);
  robot::poseController->setTarget({72, 0, M_PI, 0, 0, 0});
  chassisWait();

}
