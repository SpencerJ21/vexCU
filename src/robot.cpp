#include "robot.hpp"

namespace robot {

std::shared_ptr<kappa::XDriveChassis> chassis;
std::shared_ptr<kappa::ImuInput> imu;
std::shared_ptr<okapi::Controller> controller;
std::shared_ptr<kappa::AbstractInput<std::array<double,4>>> sensorArray;

std::shared_ptr<Odom3EncImu> odometry;
std::shared_ptr<HolonomicSlew> slewChassis;

double maxLinearSpeed{67}; // in/s
double maxAngularSpeed{5}; // rad/s

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

  robot::slewChassis = std::make_shared<HolonomicSlew>(4, 10, robot::chassis);

  robot::imu = std::make_shared<kappa::ImuInput>(9);

  robot::sensorArray = std::make_shared<kappa::ArrayConsolidator<double,4>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
    std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(0,0), 0),
    std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(0,0), 0),
    std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(0,0), 0),
    robot::imu
  });

  robot::controller = std::make_shared<okapi::Controller>();

  robot::odometry = std::make_shared<Odom3EncImu>(
    std::make_unique<okapi::PassthroughFilter>(),
    std::make_unique<okapi::PassthroughFilter>(),
    std::make_unique<okapi::PassthroughFilter>(),
    robot::sensorArray
  );

  robot::imu->calibrate();

  pros::delay(2500);

}
