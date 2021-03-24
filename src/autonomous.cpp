#include "main.h"
#include "robot.hpp"

uint16_t chassisMoveId = 0;

void chassisWait(double timeout){
  double t = pros::millis() + timeout;

  do{
    pros::delay(100);
  }while(!robot::poseController->isSettled() && pros::millis() < t);

  std::cout << "Move:  " << chassisMoveId++ << "\t" << pros::millis() + timeout - t << "\n";
}


void autonomous() {

  pros::Task chassisThread([&]{
    while(true){
      while(!robot::poseController->isSettled()){
        robot::slewChassis->set(robot::poseController->step(robot::odometry->get()));
        pros::delay(10);
      }

      while(robot::poseController->isSettled()){
        robot::poseController->step(robot::odometry->get());
        robot::slewChassis->set({0,0,0});
        pros::delay(10);
      }
    }
  }, "Auton Chassis Runner");

  pros::Task logTask([&]{
    while(true){
      auto pose = robot::odometry->get();
      std::cout << "(" << pose.x << ", " << pose.y << ", " << pose.theta << ")\n";

      //auto data = robot::sensorArray->get();
      //std::cout << data[0] << ", " << data[0] << ", " << data[0] << ", " << data[0] << "\n";
      pros::delay(100);
    }
  }, "Log Task");

  auto startPushAgainstGoal = [&]{
    chassisThread.suspend();

    robot::chassis->set({20, 0, 0});
  };

  auto endPushAgainstGoal = [&]{
    chassisThread.resume();
  };

  // deploy
  robot::intake->runBField(0b01111000);
  pros::delay(500);

  // B1 ball
  robot::intake->intake();
  robot::intake->waitForBall(1, 1000);

  robot::poseController->setTarget({26, 12, M_PI_2, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({26, 19, M_PI_2, 0, 0, 0});
  robot::intake->waitForBall(1, 3000);

  // BL goal
  robot::intake->idle();
  robot::poseController->setTarget({13, 12, M_PI_2 + M_PI_4, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({9, 19, M_PI_2 + M_PI_4, 0, 0, 0});
  chassisWait(3000);

  startPushAgainstGoal();
  robot::intake->runAll();
  robot::intake->waitForBall(2, 3000);

  // Retreat and dump
  endPushAgainstGoal();
  robot::intake->dump();
  robot::poseController->setTarget({30, -5, M_PI_2, 0, 0, 0});
  chassisWait(5000);

  // C2 ball
  robot::intake->intake();
  robot::poseController->setTarget({36, -12, 0, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({60, -12, 0, 0, 0, 0});
  chassisWait(5000);

  // C1 ball
  robot::poseController->setTarget({62, -12, M_PI_2, 0, 0, 0});
  chassisWait(5000);

  // BM goal
  robot::poseController->setTarget({62, 12, M_PI_2, 0, 0, 0});
  chassisWait(3000);

  startPushAgainstGoal();
  robot::intake->runAll();
  robot::intake->waitForBall(1, 2000);

  robot::intake->outtake();
  pros::delay(500);

  // Retreat and dump
  endPushAgainstGoal();
  robot::poseController->setTarget({62, 0, M_PI_2, 0, 0, 0});
  robot::intake->dump();
  chassisWait(3000);

  // E1 ball
  robot::poseController->setTarget({97, 8, M_PI_2 + 0.3, 0, 0, 0});
  chassisWait(5000);

  robot::intake->intake();
  robot::poseController->setTarget({97, 19, M_PI_2, 0, 0, 0});
  robot::intake->waitForBall(1, 2000);

  // BR goal
  robot::poseController->setTarget({110, 12, M_PI_4, 0, 0, 0});
  chassisWait(5000);

  robot::intake->idle();
  robot::poseController->setTarget({116, 17, M_PI_4, 0, 0, 0});
  chassisWait(3000);

  startPushAgainstGoal();
  robot::intake->runAll();
  robot::intake->waitForBall(2, 3000);

  // retreat and dump
  endPushAgainstGoal();
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

  // C4 ball
  robot::intake->intake();
  robot::poseController->setTarget({96, -59, -M_PI, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({86, -59, -M_PI, 0, 0, 0});
  chassisWait(5000);

  // C5 ball
  robot::poseController->setTarget({70, -60, -M_PI_2, 0, 0, 0});
  chassisWait(5000);

  // FM goal
  robot::poseController->setTarget({70, -82, -M_PI_2, 0, 0, 0});
  chassisWait(3000);

  robot::intake->runAll();
  robot::intake->waitForBall(1, 2000);

  // Retreat and dump
  robot::poseController->setTarget({70, -70, -M_PI_2, 0, 0, 0});
  robot::intake->dump();
  chassisWait(3000);

  // B6 ball
  robot::intake->intake();
  robot::poseController->setTarget({31, -77, -M_PI_2, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({31, -86, -M_PI_2, 0, 0, 0});
  robot::intake->waitForBall(1, 3000);

  // FL goal
  robot::intake->idle();
  robot::poseController->setTarget({22, -84, -M_PI_2 - M_PI_4, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({17.5, -87, -M_PI_2 - M_PI_4, 0, 0, 0});
  chassisWait(3000);

  robot::intake->runAll();
  robot::intake->waitForBall(2, 3000);

  // Retreat and dump
  robot::intake->dump();
  robot::poseController->setTarget({22, -84, -M_PI_2 - M_PI_4, 0, 0, 0});
  chassisWait(1000);

  robot::poseController->setTarget({24, -96, M_PI_2, 0, 0, 0});
  chassisWait(5000);

  // A5 Ball
  robot::intake->intake();
  robot::poseController->setTarget({24, -84, M_PI_2, 0, 0, 0});
  robot::intake->waitForBall(1, 3000);

  // ML Goal
  robot::poseController->setTarget({32, -36, M_PI, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({24, -36, M_PI, 0, 0, 0});
  chassisWait(3000);

  robot::intake->runAll();
  robot::intake->waitForBall(1, 2000);

  // Retreat
  robot::intake->dump();
  robot::poseController->setTarget({36, -36, M_PI, 0, 0, 0});
  chassisWait(3000);

  // B3 Ball
  robot::intake->intake();
  robot::poseController->setTarget({20, -36, 0, 0, 0, 0});
  chassisWait(3000);

  robot::poseController->setTarget({34, -36, 0, 0, 0, 0});
  robot::intake->waitForBall(1, 2000);

  // Descore mid
  robot::poseController->setTarget({42, -32, 0, 0, 0, 0});
  chassisWait(1000);

  robot::poseController->setTarget({36, -32, 0, 0, 0, 0});
  chassisWait(1000);

  robot::poseController->setTarget({42, -32, 0, 0, 0, 0});
  chassisWait(1000);

  robot::poseController->setTarget({36, -32, 0, 0, 0, 0});
  chassisWait(1000);

  robot::poseController->setTarget({42, -32, 0, 0, 0, 0});
  chassisWait(1000);

  robot::poseController->setTarget({34, -36, 0, 0, 0, 0});
  chassisWait(1000);

  // Score mid
  robot::poseController->setTarget({42, -36, 0, 0, 0, 0});
  chassisWait(2000);

  robot::intake->outtake();
  pros::delay(1500);

  robot::poseController->setTarget({34, -36, 0, 0, 0, 0});
  chassisWait(2000);


  // stop
  while(true){
    robot::chassis->set({0,0,0});
    robot::intake->idle();
  }
}
