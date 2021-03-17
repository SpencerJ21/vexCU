#include "main.h"
#include "robot.hpp"

void chassisWait(double timeout){
  double t = pros::millis() + timeout;
  do{
    robot::slewChassis->set(robot::poseController->step(robot::odometry->get()));
    pros::delay(10);
  }while(!robot::poseController->isSettled() && pros::millis() < t);
}


void autonomous() {

  // deploy
  robot::intake->runBField(0b01010000);
  pros::delay(500);

  // B2 ball
  robot::intake->intake();
  robot::poseController->setTarget({31, 5, M_PI_2, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({31, 14, M_PI_2, 0, 0, 0});
  robot::intake->waitForBall(1, 3000);

  // BL goal
  robot::intake->idle();
  robot::poseController->setTarget({22, 12, M_PI_2 + M_PI_4, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({17.5, 15, M_PI_2 + M_PI_4, 0, 0, 0});
  chassisWait(3000);

  robot::intake->runAll();
  robot::intake->waitForBall(2, 3000);

  // Retreat and dump
  robot::intake->dump();
  robot::poseController->setTarget({36, -20, M_PI_2, 0, 0, 0});
  chassisWait(5000);

  // C2 ball
  robot::intake->intake();
  robot::poseController->setTarget({48, -13, 0, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({54, -13, 0, 0, 0, 0});
  chassisWait(5000);

  // C1 ball
  robot::poseController->setTarget({70, -12, M_PI_2, 0, 0, 0});
  chassisWait(5000);

  // BM goal
  robot::poseController->setTarget({70, 10, M_PI_2, 0, 0, 0});
  chassisWait(3000);

  robot::intake->runAll();
  robot::intake->waitForBall(1, 2000);

  // Retreat and dump
  robot::poseController->setTarget({70, 0, M_PI_2, 0, 0, 0});
  robot::intake->dump();
  chassisWait(3000);

  // E1 ball
  robot::poseController->setTarget({103, 5, M_PI_2 + 0.4, 0, 0, 0});
  chassisWait(5000);

  robot::intake->intake();
  robot::poseController->setTarget({103, 14, M_PI_2, 0, 0, 0});
  robot::intake->waitForBall(1, 2000);

  // BR goal
  robot::intake->idle();
  robot::poseController->setTarget({115, 12, M_PI_4, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({119, 15, M_PI_4, 0, 0, 0});
  chassisWait(3000);

  robot::intake->runAll();
  robot::intake->waitForBall(2, 3000);

  // retreat and dump
  robot::poseController->setTarget({100, 0, M_PI_4, 0, 0, 0});
  robot::intake->dump();
  chassisWait(5000);

  // D3 ball
  robot::intake->intake();
  robot::poseController->setTarget({92, -10, -M_PI_2, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({92, -24, -M_PI_2, 0, 0, 0});
  chassisWait(3000);

  // MR goal
  robot::intake->idle();
  robot::poseController->setTarget({102, -36, 0, 0, 0, 0});
  chassisWait(3000);

  robot::poseController->setTarget({114, -36, 0, 0, 0, 0});
  chassisWait(3000);

  robot::intake->runAll();
  robot::intake->waitForBall(1, 2000);

  //retreat and dump
  robot::poseController->setTarget({102, -36, M_PI_4, 0, 0, 0});
  robot::intake->dump();
  chassisWait(3000);

  // E5 ball
  robot::poseController->setTarget({114, -72, -M_PI_2, 0, 0, 0});
  robot::intake->intake();
  robot::intake->waitForBall(1, 3000);

  // E6 ball
  robot::poseController->setTarget({101, -86, -M_PI_2, 0, 0, 0});
  robot::intake->waitForBall(1, 2000);

  // FR goal
  robot::intake->idle();
  robot::poseController->setTarget({112, -83, -M_PI_4, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({116, -89, -M_PI_4, 0, 0, 0});
  chassisWait(3000);

  robot::intake->runAll();
  robot::intake->waitForBall(2, 3000);

  // retreat and dump
  robot::poseController->setTarget({102, -72, 0, 0, 0, 0});
  robot::intake->dump();
  chassisWait(3000);

  robot::chassis->set({0,0,0});
}
