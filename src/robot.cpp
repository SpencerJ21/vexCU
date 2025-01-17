#include "robot.hpp"

extern void screenTaskFn();

namespace robot {

std::shared_ptr<kappa::XDriveChassis> chassis;
std::shared_ptr<kappa::XDriveChassis> driverChassis;
std::shared_ptr<kappa::ImuInput> imu;
std::shared_ptr<okapi::Controller> controller;
std::shared_ptr<kappa::AbstractInput<std::array<double,4>>> sensorArray;
std::shared_ptr<Intake> intake;

std::shared_ptr<Odom3EncImu> odometry;
std::shared_ptr<HolonomicSlew> slewChassis;
std::shared_ptr<HoloPoseController> poseController;

double maxLinearSpeed{67}; // in/s
double maxAngularSpeed{5}; // rad/s

}

void initialize(){
  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(std::make_unique<okapi::Timer>(), "/ser/sout", okapi::Logger::LogLevel::debug));

  robot::chassis =
    std::make_shared<kappa::XDriveChassis>(4.2, 17, // Wheel size, Chassis Width (corner-corner)
      std::make_shared<kappa::ArrayOutputClamp<double,4>>(-210, 210, // Consider maximum wheel RPM of 210
        std::make_shared<kappa::ArrayDistributor<double,4>>(std::initializer_list<std::shared_ptr<kappa::AbstractOutput<double>>>{
          std::make_shared<kappa::VPidSubController>(kappa::makeVPIDMotor(14,  {25,50,50,620})), // Port #, {kP, kD, kF, kSF}
          std::make_shared<kappa::VPidSubController>(kappa::makeVPIDMotor(15,  {25,50,50,620})),
          std::make_shared<kappa::VPidSubController>(kappa::makeVPIDMotor(-19, {25,50,50,620})),
          std::make_shared<kappa::VPidSubController>(kappa::makeVPIDMotor(-18,  {25,50,50,620}))
        })
      )
    );

  // Max slew step
  robot::slewChassis = std::make_shared<HolonomicSlew>(2, robot::chassis);

  robot::imu = std::make_shared<kappa::ImuInput>(7);

  robot::sensorArray = std::make_shared<kappa::ArrayConsolidator<double,4>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
    std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(5,6), -0.0238149606299), // Ports, Ticks-to-inches
    std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(7,8),  0.0238149606299),
    std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(3,4), -0.0238149606299),
    robot::imu
  });

  robot::intake = std::make_shared<Intake>(3,-2,8,10,2,1); // Ports

  robot::controller = std::make_shared<okapi::Controller>();

  robot::odometry = std::make_shared<Odom3EncImu>(
    6.0, // Inches from rear wheel to tracking center
    std::make_unique<okapi::PassthroughFilter>(),
    std::make_unique<okapi::PassthroughFilter>(),
    std::make_unique<okapi::PassthroughFilter>(),
    robot::sensorArray
  );

  robot::poseController = std::make_shared<HoloPoseController>(
    std::make_unique<kappa::PidController>(kappa::PidController::Gains{3,0,0,0}, // {kP, kI, kD, kF}
      okapi::TimeUtilFactory::withSettledUtilParams(1.5, 0.7, 10 * okapi::millisecond)),

    std::make_unique<kappa::PidController>(kappa::PidController::Gains{6,0,0,0}, // {kP, kI, kD, kF}
      okapi::TimeUtilFactory::withSettledUtilParams(0.2, 0.05, 10 * okapi::millisecond))
  );

  robot::poseController->setOutputLimits({-0.8*robot::maxLinearSpeed, 0, -0.8*robot::maxAngularSpeed}, {0.8*robot::maxLinearSpeed, 0, 0.8*robot::maxAngularSpeed});

  auto calibrationTime = pros::millis();

  robot::imu->calibrate();
  robot::intake->setThresholds(1900, 2500);

  pros::Task::delay_until(&calibrationTime, 2500);

  pros::Task odomTask([&]{
  	auto t = pros::millis();

  	while(true){
  		robot::odometry->step();

  		pros::Task::delay_until(&t, 10);
  	}
  }, "Odom Task");

  pros::Task screenTask([&]{
    screenTaskFn();
  }, "Screen Task");

}
