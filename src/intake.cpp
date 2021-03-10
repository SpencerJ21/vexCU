#include "intake.hpp"

Intake::Intake(std::int8_t iintakeL, std::int8_t iintakeR, std::int8_t iuptake1, std::int8_t iouttake1):
  intakeL(okapi::Motor(iintakeL)), intakeR(okapi::Motor(iintakeR)), uptake1(okapi::Motor(iuptake1)), outtake1(okapi::Motor(iouttake1))
{
  intakeL.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  intakeR.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  uptake1.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  outtake1.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
}

void Intake::runBField(uint8_t bfield){
  intakeL.moveVoltage(voltage[(bfield & 0b11000000) >> 6]);
  intakeR.moveVoltage(voltage[(bfield & 0b00110000) >> 4]);
  uptake1.moveVoltage(voltage[(bfield & 0b00001100) >> 2]);
  outtake1.moveVoltage(voltage[bfield & 0b00000011]);
}

void Intake::intake(){
  runBField(0b10101100);
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
