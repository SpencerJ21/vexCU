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

  auto lEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(3,4), -M_PI * 2.75 / 360.0);
  auto rEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(5,6),  M_PI * 2.75 / 360.0);
  auto bEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(7,8), -M_PI * 2.75 / 360.0);
  auto fEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(1,2),  M_PI * 2.75 / 360.0);
  auto imu  = std::make_shared<kappa::ImuInput>(15);

  imu->calibrate();
  pros::delay(5000);

  std::ofstream log("/usd/odomInputTelem.csv");

  auto odomInput = std::make_shared<kappa::ArrayInputLogger<double,5>>(6, ", ", ", ", "\n", log,
    std::make_shared<kappa::ArrayConsolidator<double,5>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
      lEnc, rEnc, bEnc, fEnc, imu
    })
  );

  pros::Task testController([&] {
    std::size_t count = 0;
    auto t = pros::millis();

    while (true) {

      odomInput->get();

      if (!(++count % 10)) {
        log << std::flush;
      }

      pros::Task::delay_until(&t, 1);
    }

  }, "Test Controller");

  while(true) {
    pros::delay(10);
  }

}
