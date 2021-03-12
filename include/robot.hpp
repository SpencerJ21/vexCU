#pragma once
#include "main.h"

#include "kappaAux/holonomicSlew.hpp"
#include "kappaAux/holoPoseController.hpp"
#include "kappaAux/odometry.hpp"

#include "intake.hpp"

namespace robot {

extern std::shared_ptr<kappa::XDriveChassis> chassis;
extern std::shared_ptr<kappa::XDriveChassis> driverChassis;
extern std::shared_ptr<kappa::ImuInput> imu;
extern std::shared_ptr<okapi::Controller> controller;
extern std::shared_ptr<kappa::AbstractInput<std::array<double,4>>> sensorArray;
extern std::shared_ptr<Intake> intake;

extern std::shared_ptr<Odom3EncImu> odometry;
extern std::shared_ptr<HolonomicSlew> slewChassis;
extern std::shared_ptr<HoloPoseController> poseController;

extern double maxLinearSpeed;
extern double maxAngularSpeed;

}
