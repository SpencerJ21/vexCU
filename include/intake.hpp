#include "main.h"

class Intake {
public:
  Intake(std::int8_t iintakeL, std::int8_t iintakeR, std::int8_t iuptake1, std::int8_t iouttake1, std::uint8_t isensor);

  // Blocking; ~1s
  void calibrateSensor(double detectionThresholdProportion, double clearThresholdProportion);

  bool checkForBall();

  bool checkForClear();

  void waitForBall(uint8_t numberOfBalls, uint32_t timeout);

  int32_t getSensorValue();

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

  pros::ADILineSensor ballSensor;
  int32_t detectionThreshold;
  int32_t clearThreshold;

  const int16_t voltage[4]{0, -12000, 12000, 6000};
};
