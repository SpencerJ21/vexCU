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

  std::cout << "0";
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

  auto lEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(3,4), -M_PI * 2.75 / 360.0);
  auto rEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(5,6),  M_PI * 2.75 / 360.0);
  auto bEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(7,8), -M_PI * 2.75 / 360.0);
  auto fEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(1,2),  M_PI * 2.75 / 360.0);
  auto imu  = std::make_shared<kappa::ImuInput>(15);

  auto c1 = lv_chart_create(lv_scr_act(), NULL);
  lv_obj_set_size(c1, 480, 240);
  lv_obj_set_pos(c1, 0, 0);
  lv_chart_set_type(c1, LV_CHART_TYPE_LINE);
  lv_chart_set_range(c1, -90, 90);
  lv_chart_set_point_count(c1, 240);
  auto s1 = lv_chart_add_series(c1, LV_COLOR_BLUE);
  auto s2 = lv_chart_add_series(c1, LV_COLOR_RED);

  auto filterImu = 
    std::make_shared<kappa::InputChartLogger<double>>(c1,s2,
      std::make_shared<kappa::InputFilter<double>>(std::make_unique<okapi::EKFFilter>(0.01, 0.023879594),
        std::make_shared<kappa::InputChartLogger<double>>(c1,s1,
          imu
        )
      )
    );

  imu->calibrate();
  pros::delay(5000);

  odom1 =
    std::make_shared<OdomInput>(OdomInput::OdomVals{13.425, 5.45, 0.01},
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_shared<kappa::ArrayConsolidator<double,3>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
        lEnc, bEnc, rEnc
      })
    );

  odom2 =
    std::make_shared<OdomInput4>(OdomInput4::OdomVals{13.425, 10.906, 0.01},
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_shared<kappa::ArrayConsolidator<double,4>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
        lEnc, bEnc, rEnc, fEnc
      })
    );

  odom3 =
    std::make_shared<OdomInput2Imu>(OdomInput2Imu::OdomVals{6.7126, 5.45, 0.01},
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_shared<kappa::ArrayConsolidator<double,3>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
        lEnc, bEnc, imu
      })
    );

  odom4 =
    std::make_shared<OdomInput3Imu>(OdomInput3Imu::OdomVals{13.425, 5.45, 0.01},
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_shared<kappa::ArrayConsolidator<double,4>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
        lEnc, bEnc, rEnc, imu
      })
    );

  odom5 =
    std::make_shared<OdomInput4Imu>(OdomInput4Imu::OdomVals{13.425, 10.906, 0.01},
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_unique<okapi::PassthroughFilter>(),
      std::make_shared<kappa::ArrayConsolidator<double,5>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
        lEnc, bEnc, rEnc, fEnc, imu
      })
    );

  pros::Task testController([&] {

    while (true) {
      arrPrint(odom1->step(), '1');
      arrPrint(odom2->step(), '2');
      arrPrint(odom3->step(), '3');
      arrPrint(odom4->step(), '4');
      arrPrint(odom5->step(), '5');

      std::cout << "\n";

      pros::delay(10);
    }

  }, "Test Controller");

  while(true) {
    pros::delay(10);
  }

}
