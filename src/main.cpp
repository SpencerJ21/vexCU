#include "main.h"
#include "kappaAux/odomInput.hpp"
#include "kappaAux/odomInput4.hpp"
#include "kappaAux/odomInput2Imu.hpp"
#include "kappaAux/odomInput3Imu.hpp"
#include "kappaAux/odomInput4Imu.hpp"

template <std::size_t N>
void arrPrint(const std::array<double,N> &values, char id) {
  std::cout << pros::millis() << ", #" << id << ", ";

  for(const double &i : values){
    std::cout << i << ", ";
  }

  std::cout << "0\n";
}

void initialize() {
  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(std::make_unique<okapi::Timer>(), "/ser/sout", okapi::Logger::LogLevel::debug));
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

std::shared_ptr<OdomInput> odom1;
std::shared_ptr<OdomInput4> odom2;
std::shared_ptr<OdomInput2Imu> odom3;
std::shared_ptr<OdomInput3Imu> odom4;
std::shared_ptr<OdomInput4Imu> odom5;

void opcontrol() {

  auto lEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(1,2), M_PI * 2.75 / 180.0);
  auto rEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(3,4), M_PI * 2.75 / 180.0);
  auto bEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(5,6), M_PI * 2.75 / 180.0);
  auto fEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(7,8), M_PI * 2.75 / 180.0);
  auto imu  = std::make_shared<kappa::ImuInput>(1);

  odom1 =
    std::make_shared<OdomInput>(OdomInput::OdomVals{10, 5, 0.005},
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_shared<kappa::ArrayConsolidator<double,3>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
        lEnc, bEnc, rEnc
      })
    );

  odom2 =
    std::make_shared<OdomInput4>(OdomInput4::OdomVals{10, 10, 0.005},
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_shared<kappa::ArrayConsolidator<double,4>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
        lEnc, bEnc, rEnc, fEnc
      })
    );

  odom3 =
    std::make_shared<OdomInput2Imu>(OdomInput2Imu::OdomVals{5, 5, 0.005},
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_shared<kappa::ArrayConsolidator<double,3>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
        lEnc, bEnc, imu
      })
    );

  odom4 =
    std::make_shared<OdomInput3Imu>(OdomInput3Imu::OdomVals{10, 5, 0.005},
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_shared<kappa::ArrayConsolidator<double,4>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
        lEnc, bEnc, rEnc, imu
      })
    );

  odom5 =
    std::make_shared<OdomInput4Imu>(OdomInput4Imu::OdomVals{10, 10, 0.005},
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_shared<kappa::ArrayConsolidator<double,5>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
        lEnc, bEnc, rEnc, fEnc, imu
      })
    );

  pros::Task testController([&] {
    std::uint32_t now = pros::millis();

    while (true) {
      arrPrint(odom1->step(), '1');
      arrPrint(odom2->step(), '2');
      arrPrint(odom3->step(), '3');
      arrPrint(odom4->step(), '4');
      arrPrint(odom5->step(), '5');
      std::cout << "\n";

      pros::Task::delay_until(&now, 5);
    }

  }, "Test Controller");

  while(true) {
    pros::delay(10);
  }

}
