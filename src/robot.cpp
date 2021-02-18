#include "robot.hpp"

namespace robot {

std::shared_ptr<kappa::XDriveChassis> chassis;
std::shared_ptr<XDriveSlew> slewChassis;
std::shared_ptr<kappa::ImuInput> imu;

}

void initialize(){
  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(std::make_unique<okapi::Timer>(), "/ser/sout", okapi::Logger::LogLevel::debug));

  robot::chassis =
    std::make_shared<kappa::XDriveChassis>(4.2, 17,
      std::make_shared<kappa::ArrayOutputClamp<double,4>>(-220, 220,
        std::make_shared<kappa::ArrayDistributor<double,4>>(std::initializer_list<std::shared_ptr<kappa::AbstractOutput<double>>>{
          std::make_shared<kappa::VPidSubController>(kappa::makeVPIDMotor(11,  {25,50,50,620})),
          std::make_shared<kappa::VPidSubController>(kappa::makeVPIDMotor(20,  {25,50,50,620})),
          std::make_shared<kappa::VPidSubController>(kappa::makeVPIDMotor(-10, {25,50,50,620})),
          std::make_shared<kappa::VPidSubController>(kappa::makeVPIDMotor(-1,  {25,50,50,620}))
        })
      )
    );

  robot::slewChassis = std::make_shared<XDriveSlew>(2, 8, robot::chassis);

}
