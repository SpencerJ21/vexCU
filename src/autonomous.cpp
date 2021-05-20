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

void intakeCycle(double bottom, double top, double timeout){
  auto targetBallCounts = robot::intake->getBallCounts();
  targetBallCounts.first += bottom;
  targetBallCounts.second += top;
  double t = pros::millis() + timeout;

  while(pros::millis() < t){
    switch(((targetBallCounts.first  <= robot::intake->getBallCounts().first) << 1) +
            (targetBallCounts.second <= robot::intake->getBallCounts().second)){
      case 0b00: // Awaiting balls at top and bottom sensors
        robot::intake->execute(12000,12000,12000);
        break;
      case 0b01: // Awaiting ball(s) at bottom sensor
        robot::intake->execute(12000,12000,0);
        break;
      case 0b10: // Awaiting ball(s) at top sensor
        robot::intake->execute(0,12000,12000);
        break;
      case 0b11: // Awaiting no balls
        robot::intake->execute(0,0,0);
        return;

      default:
        std::cout << "You broke it idiot\n";
        break;
    }
    pros::delay(50);
  }
}

void intakeNoMid(double bottom, double timeout){
  auto targetBallCount = robot::intake->getBallCounts().first + bottom;
  double t = pros::millis() + timeout;

  robot::intake->execute(12000, 0, 0);

  while(pros::millis() < t && targetBallCount > robot::intake->getBallCounts().first){
    pros::delay(50);
  }
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
        robot::chassis->set({12, 0, 0});
        pros::delay(10);
      }
    }
  }, "Auton Chassis Runner");

  pros::Task ballCounterThread([&]{
    bool bottomOccupied = false;
    bool topOccupied = false;

    while(true){
      if(bottomOccupied){
        if(robot::intake->checkForClearLower()){
          bottomOccupied = false;
          robot::intake->incrementBallCounter(true, false);
        }
      }else{
        if(robot::intake->checkForBallLower()){
          bottomOccupied = true;
          robot::intake->incrementBallCounter(true, false);
        }
      }

      if(topOccupied){
        if(robot::intake->checkForClearUpper()){
          topOccupied = false;
          robot::intake->incrementBallCounter(false, true);
        }
      }else{
        if(robot::intake->checkForBallUpper()){
          topOccupied = true;
          robot::intake->incrementBallCounter(false, true);
        }
      }
      pros::delay(10);
    }
  });

  pros::Task logTask([&]{
    while(true){
      auto pose = robot::odometry->get();
      auto ballCounts = robot::intake->getBallCounts();
      std::cout << pros::millis() << "\t(" << ballCounts.first << ", " << ballCounts.second << ")\n";
      //std::cout << "(" << pose.x << ", " << pose.y << ", " << pose.theta << "\t" << robot::intake->getSensorValue() << ")\n";

      //auto data = robot::sensorArray->get();
      //std::cout << data[0] << ", " << data[0] << ", " << data[0] << ", " << data[0] << "\n";
      pros::delay(100);
    }
  }, "Log Task");

  double ballCountFlag = 0;

  // deploy
  robot::intake->execute(6000, 0, 0);
  pros::delay(500);

  // B1 ball
  intakeCycle(1, 0, 1000);

  // ML goal
  robot::poseController->setTarget({18, -36, -M_PI, 0, 0, 0});
  chassisWait(3000);

  pushingAgainstGoal = true;
  intakeCycle(0.5, robot::intake->getBallCounts().first - ballCountFlag > 1.0 ? 2.0 : 1.0, 3000);

  // Dump and retreat
  pushingAgainstGoal = false;
  robot::intake->execute(-12000, -12000, -12000);
  robot::poseController->setTarget({18, -38, -M_PI, 0, 0, 0});
  chassisWait(3000);


  robot::poseController->setTarget({26, 12, -M_PI - M_PI_2, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({26, 19, -M_PI - M_PI_2, 0, 0, 0});
  intakeCycle(1,0,2000);

  // BL goal
  //robot::intake->execute(4000, 0, 0);
  robot::poseController->setTarget({13, 12, -M_PI - M_PI_4, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({9, 19, -M_PI - M_PI_4, 0, 0, 0});
  chassisWait(3000);

  pushingAgainstGoal = true;
  intakeCycle(1.5, 1, 3000);

  //robot::intake->outtake();
  //pros::delay(500);

  // Retreat and dump
  pushingAgainstGoal = false;
  robot::intake->execute(-12000, -12000, -12000);
  robot::poseController->setTarget({30, -5, 0, 0, 0, 0});
  chassisWait(5000);

  // C2 ball
  ballCountFlag = robot::intake->getBallCounts().first;
  robot::poseController->setTarget({36, -13, -0.1, 0, 0, 0});
  intakeCycle(1, 0, 2000);
  chassisWait(3000);

  robot::poseController->setTarget({60, -13, 0, 0, 0, 0});
  intakeNoMid(1, 2000);
  chassisWait(3000);

  // C1 ball
  robot::poseController->setTarget({62, -12, M_PI_2, 0, 0, 0});
  chassisWait(5000);

  // BM goal
  robot::poseController->setTarget({62, 12, M_PI_2, 0, 0, 0});
  chassisWait(3000);

  pushingAgainstGoal = true;
  intakeCycle(0.5, robot::intake->getBallCounts().first - ballCountFlag == 2.0 ? 2.0 : 1.0, 2000);

  //robot::intake->outtake();
  //pros::delay(500);

  // Retreat and dump
  pushingAgainstGoal = false;
  robot::poseController->setTarget({62, 0, 2*M_PI/3, 0, 0, 0});
  robot::intake->execute(-12000, -12000, -12000);
  chassisWait(3000);

  // E1 ball
  robot::poseController->setTarget({97, 8, 2*M_PI/3, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({97, 19, M_PI_2, 0, 0, 0});
  intakeCycle(1, 0, 2000);

  // BR goal
  robot::poseController->setTarget({110, 12, M_PI_4, 0, 0, 0});
  chassisWait(3000);

  robot::intake->execute(0, 0, 0);
  robot::poseController->setTarget({116, 17, M_PI_4, 0, 0, 0});
  chassisWait(3000);

  pushingAgainstGoal = true;
  intakeCycle(2, 1, 3000);
  robot::intake->execute(6000, 0, 0);

  //robot::intake->outtake();

  // retreat and dump
  pushingAgainstGoal = false;
  robot::poseController->setTarget({100, 0, M_PI_4, 0, 0, 0});
  robot::intake->execute(-12000, -12000, -12000);
  chassisWait(5000);

  // D3 ball
  robot::intake->execute(12000, 0, 0);
  robot::poseController->setTarget({84, -12, -M_PI_2, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({84, -32, -M_PI_2, 0, 0, 0});
  chassisWait(3000);

  // MR goal
  robot::intake->execute(6000, 0, 0);
  robot::poseController->setTarget({107, -36, 0, 0, 0, 0});
  chassisWait(3000);

  pushingAgainstGoal = true;
  pros::delay(500);

  intakeCycle(0.5, 1, 2000);

  //robot::intake->runBField(0b01011010);
  //pros::delay(500);

  //retreat and dump
  pushingAgainstGoal = false;
  robot::poseController->setTarget({100, -36, M_PI/6, 0, 0, 0});
  robot::intake->execute(-12000, -12000, -12000);
  chassisWait(3000);

  // E5 ball
  ballCountFlag = robot::intake->getBallCounts().first;
  robot::poseController->setTarget({111, -50, -M_PI_2, 0, 0, 0});
  chassisWait(3000);

  robot::poseController->setTarget({111, -65, -M_PI_2, 0, 0, 0});
  robot::intake->execute(12000, 12000, 0);
  chassisWait(3000);

  // E6 ball
  robot::poseController->setTarget({100, -91, -M_PI_2, 0, 0, 0});
  pros::delay(500);
  intakeNoMid(1, 1500);

  // FR goal
  robot::poseController->setTarget({111, -80, -M_PI_4, 0, 0, 0});
  chassisWait(5000);

  robot::intake->execute(0, 0, 0);
  robot::poseController->setTarget({118, -88, -M_PI_4, 0, 0, 0});
  chassisWait(3000);

  pushingAgainstGoal = true;
  intakeCycle(1.5, robot::intake->getBallCounts().first - ballCountFlag == 2.0 ? 2.0 : 1.0, 3000);

  //robot::intake->outtake();

  // Dump and retreat
  pushingAgainstGoal = false;
  robot::intake->execute(-12000, -12000, -12000);
  robot::poseController->setTarget({100, -72, -M_PI_4, 0, 0, 0});
  chassisWait(3000);

  // C4 ball
  ballCountFlag = robot::intake->getBallCounts().first;
  robot::intake->execute(12000, 12000, 0);
  robot::poseController->setTarget({83, -58, -M_PI, 0, 0, 0});
  chassisWait(5000);

  robot::poseController->setTarget({64, -58, -M_PI, 0, 0, 0});
  chassisWait(5000);

  // C5 ball
  pros::delay(500);

  robot::intake->execute(12000, 0, 0);
  robot::poseController->setTarget({62, -82, -M_PI_2, 0, 0, 0});
  chassisWait(5000);

  // FM goal
  pushingAgainstGoal = true;
  pros::delay(500);

  intakeCycle(0.5, robot::intake->getBallCounts().first - ballCountFlag == 2.0 ? 2.0 : 1.0, 2000);

  //robot::intake->outtake();
  //pros::delay(500);

  // Retreat and dump
  pushingAgainstGoal = false;
  robot::poseController->setTarget({62, -75, -M_PI/3, 0, 0, 0});
  robot::intake->execute(-12000, -12000, -12000);
  chassisWait(3000);
  // A5 ball
  robot::intake->execute(0, 0, 0);
  robot::poseController->setTarget({35, -72, -M_PI, 0, 0, 0});
  chassisWait(3000);

  robot::poseController->setTarget({22, -72, -M_PI, 0, 0, 0});
  intakeCycle(1, 0, 2000);

  // FL goal
  robot::poseController->setTarget({13, -91, -M_PI_2 - M_PI_4, 0, 0, 0});
  chassisWait(3000);

  pushingAgainstGoal = true;
  intakeCycle(0, 1, 2000);
  pros::delay(300);

  // Retreat and dump
  pushingAgainstGoal = false;
  robot::poseController->setTarget({26, -77, -M_PI_2, 0, 0, 0});
  robot::intake->execute(-12000, -12000, -12000);
  chassisWait(3000);

  // B6 ball
  robot::poseController->setTarget({25, -91, -M_PI_2, 0, 0, 0});
  intakeCycle(1, 0, 2000);

  // Middle goal
  robot::poseController->setTarget({43, -36, 0, 0, 0, 0});
  chassisWait(5000);

  pushingAgainstGoal = true;
  intakeCycle(1, 0, 5000);

  intakeCycle(1.5, 1, 3000);

  robot::intake->execute(0, 0, 0);
  pushingAgainstGoal = false;
  robot::poseController->setTarget({36, -36, 0, 0, 0, 0});
  chassisWait(5000);

  // stop
  robot::chassis->stop();
}
