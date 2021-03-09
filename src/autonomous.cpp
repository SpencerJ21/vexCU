#include "main.h"
#include "robot.hpp"

inline void chassisWait(){
  while(!robot::poseController->isSettled()){
    robot::slewChassis->set(robot::poseController->step(robot::odometry->get()));
    pros::delay(10);
  }
}


void autonomous() {

  pros::Task odomTask([&]{
    auto t = pros::millis();

    while(true){
      robot::odometry->step();

      pros::Task::delay_until(&t, 10);
    }
  }, "Odom Task");

  robot::intake->runBField(0b01010000);
  pros::delay(300);

  robot::intake->intake();

  robot::poseController->setStopping(false);
  robot::poseController->setTarget({34, 16, M_PI_2, 0, 0, 0});
  pros::delay(1500);

  robot::intake->runBField(0b11111100);
  chassisWait();

  robot::poseController->setStopping(true);
  robot::poseController->setTarget({34, 20, M_PI_2, 0, 0, 0});
  chassisWait();

  robot::intake->idle();
  robot::poseController->setStopping(false);
  robot::poseController->setTarget({18, 18, M_PI_2 + M_PI_4, 0, 0, 0});
  chassisWait();

  robot::poseController->setStopping(true);
  robot::poseController->setTarget({15, 21, M_PI_2 + M_PI_4, 0, 0, 0});
  chassisWait();

  robot::intake->runAll();
  pros::delay(1500);

  robot::intake->dump();
  robot::poseController->setStopping(false);
  robot::poseController->setTarget({36, 0, M_PI_2 + M_PI_4, 0, 0, 0});
  chassisWait();

  robot::intake->intake();
  robot::poseController->setStopping(true);
  robot::poseController->setTarget({54, -12, 0, 0, 0, 0});
  chassisWait();

  robot::poseController->setStopping(false);
  robot::poseController->setTarget({72, -6, M_PI_2, 0, 0, 0});
  chassisWait();

  robot::poseController->setStopping(true);
  robot::poseController->setTarget({72, 18, M_PI_2, 0, 0, 0});
  chassisWait();

  robot::intake->runAll();
  pros::delay(1500);

  robot::intake->dump();
  robot::poseController->setStopping(true);
  robot::poseController->setTarget({72, 0, M_PI, 0, 0, 0});
  chassisWait();

}
