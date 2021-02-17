#include "robot.hpp"

namespace robot {

std::shared_ptr<kappa::ThreeAxisChassis> chassis;
std::shared_ptr<kappa::ImuInput> imu;

}

void initialize(){
  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(std::make_unique<okapi::Timer>(), "/ser/sout", okapi::Logger::LogLevel::debug));

  robot::chassis = std::make_shared<kappa::ThreeAxisChassis>(4.2, 17,
    std::make_shared<kappa::ArrayDistributor<double,4>>(std::initializer_list<std::shared_ptr<kappa::AbstractOutput<double>>>{
      std::make_shared<kappa::VPidSubController>(kappa::makeVPIDMotor(0, {0,0,50,620})),
      std::make_shared<kappa::VPidSubController>(kappa::makeVPIDMotor(0, {0,0,50,620})),
      std::make_shared<kappa::VPidSubController>(kappa::makeVPIDMotor(0, {0,0,50,620})),
      std::make_shared<kappa::VPidSubController>(kappa::makeVPIDMotor(0, {0,0,50,620}))
    })
  );

  robot::imu = std::make_shared<kappa::ImuInput>(0);

}
