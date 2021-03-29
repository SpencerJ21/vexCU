#include "intake.hpp"

Intake::Intake(std::int8_t iintakeL, std::int8_t iintakeR, std::int8_t iuptake1, std::int8_t iouttake1, std::uint8_t isensor):
  intakeL(okapi::Motor(iintakeL)), intakeR(okapi::Motor(iintakeR)), uptake1(okapi::Motor(iuptake1)), outtake1(okapi::Motor(iouttake1)), ballSensor(pros::ADILineSensor(isensor))
{
  intakeL.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  intakeR.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  uptake1.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  outtake1.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
}

void Intake::setThresholds(double idetectionThreshold, double iclearThreshold){
  detectionThreshold = idetectionThreshold;
  clearThreshold     = iclearThreshold;
}

std::pair<int32_t, int32_t> Intake::getThresholds(){
  return {clearThreshold, detectionThreshold};
}

bool Intake::checkForBall(){
  std::cout << "O\t" << ballSensor.get_value() << "\n";
  return ballSensor.get_value() < detectionThreshold;
}

bool Intake::checkForClear(){
  std::cout << "X\t" << ballSensor.get_value() << "\n";
  return ballSensor.get_value() > clearThreshold;
}

void Intake::waitForBall(uint8_t numberOfBalls, uint32_t timeout, bool assertClear){
  uint32_t maxTime = pros::millis() + timeout;

  // ensure intake is clear
  while(!checkForClear() && pros::millis() < maxTime){
    pros::delay(10);
  }

  // wait until ball is detected
  while(!checkForBall() && pros::millis() < maxTime){
    pros::delay(10);
  }

  // wait until ball clears sensor if specified
  while(!checkForClear() && pros::millis() < maxTime && (assertClear || numberOfBalls > 1)){
    pros::delay(10);
  }

  if(numberOfBalls == 1){
    return;
  }else{
    waitForBall(numberOfBalls - 1, maxTime - pros::millis(), assertClear);
  }
}

int32_t Intake::getSensorValue(){
  return ballSensor.get_value();
}

void Intake::runBField(uint8_t bfield){
  intakeL.moveVoltage(voltage[(bfield & 0b11000000) >> 6]);
  intakeR.moveVoltage(voltage[(bfield & 0b00110000) >> 4]);
  uptake1.moveVoltage(voltage[(bfield & 0b00001100) >> 2]);
  outtake1.moveVoltage(voltage[bfield & 0b00000011]);
}

void Intake::intake(){
  runBField(0b10101000);
}

void Intake::outtake(){
  runBField(0b00001010);
}

void Intake::runAll(){
  runBField(0b10101010);
}

void Intake::dump(){
  runBField(0b01010101);
}

void Intake::idle(){
  runBField(0b00000000);
}
