#include "kappaAux/holonomicSlew.hpp"
#include <cmath>

HolonomicSlew::HolonomicSlew(double ispdSlewStep, double idirSlewStep, std::shared_ptr<kappa::XDriveChassis> ichassis):
    spdSlewStep(ispdSlewStep), dirSlewStep(idirSlewStep), chassis(ichassis){}

void HolonomicSlew::set(const std::tuple<double,double,double> &itarget){
  double spdDiff = std::get<0>(itarget) - std::get<0>(out);

  // normal slew alg
  std::get<0>(out) += spdDiff > 0 ?
          std::min(spdDiff, spdSlewStep) : std::max(spdDiff, -spdSlewStep);

  // angle difference on [-pi,pi]
  double angDiff = atan2( sin(std::get<1>(itarget) - std::get<1>(out)),
                          cos(std::get<1>(itarget) - std::get<1>(out)));

  // scale slew value inversely with speed (infinite directional slew when static)
  std::get<1>(out) += angDiff > 0 ?
          std::min(angDiff,  dirSlewStep / std::get<0>(out)):
          std::max(angDiff, -dirSlewStep / std::get<0>(out));

  // if target speed is nonzero, preserve curvature. Otherwise, preserve rotation
  std::get<2>(out) = std::get<0>(itarget) != 0 ?
                          std::get<2>(itarget) * (std::get<0>(out) / std::get<0>(itarget)):
                          std::get<2>(itarget);

  chassis->setPolar(out);
}

void HolonomicSlew::stop(){
  chassis->stop();
}

std::shared_ptr<kappa::XDriveChassis> HolonomicSlew::getOutput() const {
  return chassis;
}
