#include "main.h"

class Intake {
public:
  Intake(std::int8_t iintakeL, std::int8_t iintakeR, std::int8_t iuptake1, std::int8_t iouttake1);

  void runBField(uint8_t bfield);

  void intake();

  void outtake();

  void runAll();

  void dump();

  void idle();

private:
  okapi::Motor intakeL;
  okapi::Motor intakeR;
  okapi::Motor uptake1;
  okapi::Motor outtake1;

  const int16_t voltage[4]{0, -12000, 12000, 6000};
};
