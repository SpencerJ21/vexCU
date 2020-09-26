#include "kappaAux/odomInput2Imu.hpp"
#include <cmath>

OdomInput2Imu::OdomInput2Imu(OdomVals &&ivals,
          std::unique_ptr<okapi::Filter> ivelFilter,
          std::unique_ptr<okapi::Filter> istfVelFilter,
          std::unique_ptr<okapi::Filter> iangVelFilter,
          std::shared_ptr<kappa::AbstractOutput<std::array<double,6>>> ioutput):
          output(ioutput), vals(ivals), velFilter(std::move(ivelFilter)),
          stfVelFilter(std::move(istfVelFilter)),
          angVelFilter(std::move(iangVelFilter))
  {
    lastIn[3] = pros::millis();
  }

void OdomInput2Imu::set(const std::array<double,5> &input) {
  double dL     = input[0] - lastIn[0];
  double dM     = input[2] - lastIn[1];
  double dTheta = input[4] * M_PI / 180 - lastIn[2];
  double dT = pros::millis() - lastIn[3];

  if(dT == 0) return;

  lastIn[3] = pros::millis();
  lastIn[0] = input[0];
  lastIn[1] = input[2];
  lastIn[2] = input[4] * M_PI / 180;

  if(dTheta == 0){
    double lsin = sin(pose[2]);
    double lcos = cos(pose[2]);

    pose[0] += lcos * dL -
               lsin * dM;
    pose[1] += lcos * dM +
               lsin * dL;

    pose[3] = velFilter->   filter(dL / dT);
    pose[4] = stfVelFilter->filter(dM / dT);
    pose[5] = angVelFilter->filter(0);
  }else{
    double lsin = sin(pose[2] + 0.5 * dTheta);
    double lcos = cos(pose[2] + 0.5 * dTheta);
    double cOffset = 2 * sin(dTheta / 2);

    double A_r = dL / dTheta + vals.sideTrackingDist;
    double M_r = dM / dTheta + vals.rearTrackingDist;

    pose[0] += cOffset * (lcos * A_r - lsin * M_r);
    pose[1] += cOffset * (lcos * M_r + lsin * A_r);
    pose[2] += dTheta;
    pose[3] = velFilter->   filter(A_r * dTheta / dT);
    pose[4] = stfVelFilter->filter(M_r * dTheta / dT);
    pose[5] = angVelFilter->filter(      dTheta / dT);
  }

  output->set(pose);
}

const std::array<double,6> &OdomInput2Imu::get() const {
  return pose;
}
