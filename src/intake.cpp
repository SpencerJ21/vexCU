#include "intake.hpp"

Intake::Intake(std::int8_t iintakeL, std::int8_t iintakeR, std::int8_t iuptake1, std::int8_t iouttake1, std::uint8_t ilowerSensor, std::uint8_t iupperSensor):
  intakeL(okapi::Motor(iintakeL)), intakeR(okapi::Motor(iintakeR)), uptake1(okapi::Motor(iuptake1)), outtake1(okapi::Motor(iouttake1)), lowerBallSensor(pros::ADILineSensor(ilowerSensor)), upperBallSensor(pros::ADILineSensor(iupperSensor))
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

bool Intake::checkForBallLower(){
  //std::cout << "L(O)\t" << lowerBallSensor.get_value() << "\n";
  return lowerBallSensor.get_value() < detectionThreshold;
}

bool Intake::checkForClearLower(){
  //std::cout << "L(X)\t" << lowerBallSensor.get_value() << "\n";
  return lowerBallSensor.get_value() > clearThreshold;
}

bool Intake::checkForBallUpper(){
  //std::cout << "U(O)\t" << upperBallSensor.get_value() << "\n";
  return upperBallSensor.get_value() < detectionThreshold;
}

bool Intake::checkForClearUpper(){
  //std::cout << "U(X)\t" << upperBallSensor.get_value() << "\n";
  return upperBallSensor.get_value() > clearThreshold;
}

void Intake::incrementBallCounter(bool lowerEvent, bool upperEvent){
  // Increment by 0.5 for all events (rising cases and falling cases)
  // In effect, whole numbers represent balls that have passed through
  // and half counts represent a ball currently in front of sensor

  lowerCounter += lowerEvent ? 0.5 : 0;
  upperCounter += upperEvent ? 0.5 : 0;
}

std::pair<double,double> Intake::getBallCounts(){
  return {lowerCounter, upperCounter};
}

std::pair<int32_t,int32_t> Intake::getSensorValues(){
  return {lowerBallSensor.get_value(), upperBallSensor.get_value()};
}

void Intake::execute(double intakeV, double uptakeV, double outtakeV){
  if(intakeV != -1){
    intakeL.moveVoltage(intakeV);
    intakeR.moveVoltage(intakeV);
  }

  if(uptakeV != -1){
    uptake1.moveVoltage(uptakeV);
  }

  if(outtakeV != -1){
    outtake1.moveVoltage(outtakeV);
  }
}
