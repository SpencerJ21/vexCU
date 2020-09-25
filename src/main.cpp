#include "main.h"
#include "kappaAux/odomInput.hpp"
#include "kappaAux/odomInput4.hpp"
#include "kappaAux/odomInput2Imu.hpp"
#include "kappaAux/odomInput3Imu.hpp"
#include "kappaAux/odomInput4Imu.hpp"
#include <fstream>

void initialize() {
  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(std::make_unique<okapi::Timer>(), "/ser/sout", okapi::Logger::LogLevel::debug));
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {

  auto input =
    std::make_shared<kappa::ArrayInputLogger<double,5>>(
      std::make_shared<kappa::FileInput<5>>("/usd/odomInputTelem.csv")
    );

  auto t = pros::millis();

  while(true) {
    input->get();
    pros::Task::delay_until(&t, 2);
  }

}
