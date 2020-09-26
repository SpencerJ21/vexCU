#include "main.h"
#include "kappaAux/odomInput.hpp"

void initialize() {
  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(std::make_unique<okapi::Timer>(), "/ser/sout", okapi::Logger::LogLevel::debug));
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {

  auto data = std::make_shared<kappa::FileInput<5>>("/usd/odomInputTelem.csv");
  auto odom1 = std::make_shared<OdomInput>(OdomInput::OdomVals{13.3125, 0},
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_shared<kappa::ArrayOutputLogger<double,6>>()
    );

  auto t = pros::millis();

  auto count = 0;

  while(true) {
    data->get();

    if (!(count % 1)) {
      odom1->set(data->getValue());
    }

    count++;
    pros::Task::delay_until(&t, 2);
  }

}
