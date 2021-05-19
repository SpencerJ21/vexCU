#include "main.h"

class Intake {
public:
  Intake(std::int8_t iintakeL, std::int8_t iintakeR, std::int8_t iuptake1, std::int8_t iouttake1, std::uint8_t ibottomSensor, std::uint8_t iupperSensor);

  void setThresholds(double idetectionThreshold, double iclearThreshold);

  std::pair<int32_t, int32_t> getThresholds();

  bool checkForBallLower();

  bool checkForClearLower();

  bool checkForBallUpper();

  bool checkForClearUpper();

  void incrementBallCounter(bool lowerEvent, bool upperEvent);

  std::pair<double,double> getBallCounts();

  std::pair<int32_t,int32_t> getSensorValues();

  // set -1 to not update
  void execute(double intakeV, double uptakeV, double outtakeV);

private:
  okapi::Motor intakeL;
  okapi::Motor intakeR;
  okapi::Motor uptake1;
  okapi::Motor outtake1;

  pros::ADILineSensor lowerBallSensor;
  pros::ADILineSensor upperBallSensor;
  int32_t detectionThreshold;
  int32_t clearThreshold;

  double lowerCounter{1};
  double upperCounter{0};
};
