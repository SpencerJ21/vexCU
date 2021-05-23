#include "kappaAux/holonomicSlew.hpp"
#include <cmath>

HolonomicSlew::HolonomicSlew(double ispdSlewStep, std::shared_ptr<kappa::XDriveChassis> ichassis):
    spdSlewStep(ispdSlewStep), chassis(ichassis){}

void HolonomicSlew::set(const std::tuple<double,double,double> &itarget){
  // Difference in speed target
  double spdDiff = std::get<0>(itarget) - std::get<0>(out);

  // Standard slew alg
  std::get<0>(out) += spdDiff > 0 ?
          std::min(spdDiff, spdSlewStep) : std::max(spdDiff, -spdSlewStep);

  // Set direction
  std::get<1>(out) = std::get<1>(itarget);

  // If target speed is nonzero, preserve curvature. Otherwise, preserve rotation
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
