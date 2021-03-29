#include "main.h"
#include "robot.hpp"

bool pushingAgainstGoal = false;

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
      while(!robot::poseController->isSettled() && !pushingAgainstGoal){
        robot::slewChassis->set(robot::poseController->step(robot::odometry->get()));
        pros::delay(10);
      }

      while(robot::poseController->isSettled() && !pushingAgainstGoal){
        robot::poseController->step(robot::odometry->get());
        robot::slewChassis->set({0,0,0});
        pros::delay(10);
      }

      while(pushingAgainstGoal){
        robot::chassis->set({15, 0, 0});
        pros::delay(10);
      }
    }
  }, "Auton Chassis Runner");

  pros::Task logTask([&]{
    while(true){
      auto pose = robot::odometry->get();
      //std::cout << "(" << pose.x << ", " << pose.y << ", " << pose.theta << "\t" << robot::intake->getSensorValue() << ")\n";

      //auto data = robot::sensorArray->get();
      //std::cout << data[0] << ", " << data[0] << ", " << data[0] << ", " << data[0] << "\n";
      pros::delay(100);
    }
  }, "Log Task");

  // deploy
  robot::intake->runBField(0b10101000);
  pros::delay(500);

  // B1 ball
  robot::intake->runBField(0b10101101);
  robot::intake->waitForBall(1, 1000, true);
/*
  robot::poseController->setTarget({26, 12, M_PI_2, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({26, 19, M_PI_2, 0, 0, 0});
  robot::intake->waitForBall(1, 3000, true);
*/
  // BL goal
  robot::intake->runBField(0b10100000);
  robot::poseController->setTarget({13, 12, M_PI_2 + M_PI_4, 0, 0, 0});
  chassisWait(5000);

  robot::intake->idle();
  robot::poseController->setTarget({9, 19, M_PI_2 + M_PI_4, 0, 0, 0});
  chassisWait(3000);

  pushingAgainstGoal = true;
  robot::intake->runAll();
  robot::intake->waitForBall(2, 3000, true);

  robot::intake->outtake();
  pros::delay(500);

  // Retreat and dump
  pushingAgainstGoal = false;
  robot::intake->dump();
  robot::poseController->setTarget({30, -5, M_PI_2, 0, 0, 0});
  chassisWait(5000);

  // C2 ball
  robot::intake->intake();
  robot::poseController->setTarget({36, -13, -0.1, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({60, -13, 0, 0, 0, 0});
  chassisWait(5000);

  // C1 ball
  robot::poseController->setTarget({62, -12, M_PI_2, 0, 0, 0});
  chassisWait(5000);

  // BM goal
  robot::poseController->setTarget({62, 12, M_PI_2, 0, 0, 0});
  chassisWait(3000);

  pushingAgainstGoal = true;
  robot::intake->runAll();
  robot::intake->waitForBall(1, 2000);

  robot::intake->outtake();
  pros::delay(500);

  // Retreat and dump
  pushingAgainstGoal = false;
  robot::poseController->setTarget({62, 0, 2*M_PI/3, 0, 0, 0});
  robot::intake->dump();
  chassisWait(3000);

  // E1 ball
  robot::poseController->setTarget({97, 8, 2*M_PI/3, 0, 0, 0});
  chassisWait(5000);

  robot::intake->intake();
  robot::poseController->setTarget({97, 19, M_PI_2, 0, 0, 0});
  robot::intake->waitForBall(1, 2000, true);

  // BR goal
  robot::poseController->setTarget({110, 12, M_PI_4, 0, 0, 0});
  chassisWait(3000);

  robot::intake->idle();
  robot::poseController->setTarget({116, 17, M_PI_4, 0, 0, 0});
  chassisWait(3000);

  pushingAgainstGoal = true;
  robot::intake->runAll();
  robot::intake->waitForBall(2, 3000, true);

  //robot::intake->outtake();

  // retreat and dump
  pushingAgainstGoal = false;
  robot::poseController->setTarget({100, 0, M_PI_4, 0, 0, 0});
  robot::intake->dump();
  chassisWait(5000);

  // D3 ball
  robot::intake->intake();
  robot::poseController->setTarget({84, -12, -M_PI_2, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({84, -32, -M_PI_2, 0, 0, 0});
  chassisWait(3000);

  // MR goal
  robot::intake->idle();
  robot::poseController->setTarget({107, -36, 0, 0, 0, 0});
  chassisWait(3000);

  pushingAgainstGoal = true;
  pros::delay(500);

  robot::intake->runAll();
  robot::intake->waitForBall(1, 2000);

  robot::intake->runBField(0b01011010);
  pros::delay(500);

  //retreat and dump
  pushingAgainstGoal = false;
  robot::poseController->setTarget({100, -36, M_PI/6, 0, 0, 0});
  robot::intake->dump();
  chassisWait(3000);

  // E5 ball
  robot::poseController->setTarget({111, -50, -M_PI_2, 0, 0, 0});
  chassisWait(3000);

  robot::poseController->setTarget({111, -65, -M_PI_2, 0, 0, 0});
  robot::intake->intake();
  robot::intake->waitForBall(1, 3000, true);

  // E6 ball
  robot::poseController->setTarget({100, -91, -M_PI_2, 0, 0, 0});
  robot::intake->waitForBall(1, 2000, true);

  // FR goal
  robot::poseController->setTarget({110, -80, -M_PI_4, 0, 0, 0});
  chassisWait(5000);

  robot::intake->idle();
  robot::poseController->setTarget({118, -88, -M_PI_4, 0, 0, 0});
  chassisWait(3000);

  pushingAgainstGoal = true;
  robot::intake->runAll();
  robot::intake->waitForBall(2, 3000, true);

  //robot::intake->outtake();

  // Dump and retreat
  pushingAgainstGoal = false;
  robot::intake->dump();
  robot::poseController->setTarget({100, -72, -M_PI_4, 0, 0, 0});
  chassisWait(3000);

  // C4 ball
  robot::intake->intake();
  robot::poseController->setTarget({83, -58, -M_PI, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({64, -58, -M_PI, 0, 0, 0});
  chassisWait(5000);

  // C5 ball
  robot::poseController->setTarget({62, -82, -M_PI_2, 0, 0, 0});
  chassisWait(5000);

  // FM goal
  pushingAgainstGoal = true;
  pros::delay(500);

  robot::intake->runAll();
  robot::intake->waitForBall(1, 2000);

  robot::intake->outtake();
  pros::delay(500);

  // Retreat and dump
  pushingAgainstGoal = false;
  robot::poseController->setTarget({62, -75, -M_PI/3, 0, 0, 0});
  robot::intake->dump();
  chassisWait(3000);

  // B6 ball
  robot::intake->intake();
  robot::poseController->setTarget({24, -77, -M_PI/3, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({25, -91, -M_PI_2, 0, 0, 0});
  robot::intake->waitForBall(1, 3000, true);

  // Middle goal
  robot::poseController->setTarget({43, -36, 0, 0, 0, 0});
  chassisWait(5000);

  pushingAgainstGoal = true;
  robot::intake->waitForBall(1, 5000);

  robot::intake->runAll();
  robot::intake->waitForBall(1, 500);

  robot::intake->intake();
  robot::intake->waitForBall(1, 3000);

  robot::intake->idle();
  pushingAgainstGoal = false;
  robot::poseController->setTarget({36, -36, 0, 0, 0, 0});
  chassisWait(5000);

  // stop
  robot::chassis->stop();
}
