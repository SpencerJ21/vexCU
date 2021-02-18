#pragma once
#include "main.h"

#include "kappaAux/XDriveSlew.hpp"

namespace robot {

extern std::shared_ptr<kappa::XDriveChassis> chassis;
extern std::shared_ptr<XDriveSlew> slewChassis;
extern std::shared_ptr<kappa::ImuInput> imu;

}
