#pragma once
#include "main.h"

#include "kappaAux/holonomicSlew.hpp"

namespace robot {

extern std::shared_ptr<kappa::XDriveChassis> chassis;
extern std::shared_ptr<HolonomicSlew> slewChassis;
extern std::shared_ptr<kappa::ImuInput> imu;
extern std::shared_ptr<okapi::Controller> controller;

extern double maxLinearSpeed;
extern double maxAngularSpeed;

}
