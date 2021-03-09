#include "intake.hpp"

Intake::Intake(std::int8_t iintakeL, std::int8_t iintakeR, std::int8_t iuptake1, std::int8_t iouttake1):
  intakeL(okapi::Motor(iintakeL)), intakeR(okapi::Motor(iintakeR)), uptake1(okapi::Motor(iuptake1)), outtake1(okapi::Motor(iouttake1))
{
  intakeL.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  intakeR.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  uptake1.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  outtake1.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
}

void Intake::runBField(int8_t bfield){
  intakeL.moveVoltage( bfield & 0b00001000 ? (bfield > 0 ? 12000 : -12000) : 0);
  intakeR.moveVoltage( bfield & 0b00000100 ? (bfield > 0 ? 12000 : -12000) : 0);
  uptake1.moveVoltage( bfield & 0b00000010 ? (bfield > 0 ? 12000 : -12000) : 0);
  outtake1.moveVoltage(bfield & 0b00000001 ? (bfield > 0 ? 12000 : -12000) : 0);
}

void Intake::intake(){
  runBField(0b1110);
}

void Intake::outtake(){
  runBField(0b0011);
}

void Intake::runAll(){
  runBField(0b1111);
}

void Intake::dump(){
  runBField(-0b1111);
}

void Intake::idle(){
  runBField(0b0000);
}
