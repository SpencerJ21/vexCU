#include "main.h"
#include "kappaAux/odomInput.hpp"
#include "kappaAux/odomInput2Imu.hpp"
#include "kappaAux/odomInput3Imu.hpp"
#include "kappaAux/odomInput4.hpp"
#include "kappaAux/odomInput4Imu.hpp"


void initialize() {
  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(std::make_unique<okapi::Timer>(), "/ser/sout", okapi::Logger::LogLevel::debug));
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {

  auto data = std::make_shared<kappa::FileInput<5>>("/usd/odomInputTelem.csv");
  auto odom1 = std::make_shared<OdomInput>(OdomInput::OdomVals{13.3125, 5.4375},
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_shared<kappa::ArrayOutputLogger<double,6>>()
    );

  auto odom2 =
      std::make_shared<OdomInput4>(OdomInput4::OdomVals{13.3125, 10.875},
        std::make_unique<okapi::PassthroughFilter>(),
        std::make_unique<okapi::PassthroughFilter>(),
        std::make_unique<okapi::PassthroughFilter>(),
        std::make_shared<kappa::ArrayOutputLogger<double,6>>()
      );

  auto odom3 =
      std::make_shared<OdomInput2Imu>(OdomInput2Imu::OdomVals{6.65625, 5.4375},
        std::make_unique<okapi::PassthroughFilter>(),
        std::make_unique<okapi::PassthroughFilter>(),
        std::make_unique<okapi::PassthroughFilter>(),
        std::make_shared<kappa::ArrayOutputLogger<double,6>>()
      );

  auto odom4 =
      std::make_shared<OdomInput3Imu>(OdomInput3Imu::OdomVals{13.3125, 5.4375},
        std::make_unique<okapi::PassthroughFilter>(),
        std::make_unique<okapi::PassthroughFilter>(),
        std::make_unique<okapi::PassthroughFilter>(),
        std::make_shared<kappa::ArrayOutputLogger<double,6>>()
      );

  auto odom5 =
      std::make_shared<OdomInput4Imu>(OdomInput4Imu::OdomVals{13.3125, 10.875},
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
      //odom1->set(data->getValue()); // Decent Performance (.76, .52, .005)
      //odom2->set(data->getValue()); // Good Performance (-.55, -.47, .019)
      //odom3->set(data->getValue()); // Decent Performance (.61, .06, .001) (known to be unreliable in certain scenarios)
      //odom4->set(data->getValue()); // Disappointing Performance (1.68, .66, 0)
      //odom5->set(data->getValue()); // Disappointing Performance (1.00, 1.06, 0) (can be improved with sensor fusion or removing resolution optimizer)
    }

    count++;
    pros::Task::delay_until(&t, 2);
  }

}
