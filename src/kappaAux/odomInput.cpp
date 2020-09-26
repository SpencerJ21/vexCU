#include "kappaAux/odomInput.hpp"
#include <cmath>

OdomInput::OdomInput(OdomVals &&ivals,
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

void OdomInput::set(const std::array<double,5> &input) {

  double dL = input[0] - lastIn[0];
  double dM = input[2] - lastIn[1];
  double dR = input[1] - lastIn[2];
  double dT = pros::millis() - lastIn[3];

  if(dT == 0) return;

  lastIn[3] = pros::millis();
  lastIn[0] = input[0];
  lastIn[1] = input[2];
  lastIn[2] = input[1];


  if(dL == dR){
    double lsin = sin(pose[2]);
    double lcos = cos(pose[2]);

    pose[0] += lcos * dR -
               lsin * dM;
    pose[1] += lcos * dM +
               lsin * dR;

    pose[3] = velFilter->   filter(dR / dT);
    pose[4] = stfVelFilter->filter(dM / dT);
    pose[5] = angVelFilter->filter(0);
  }else{
    double dTheta = (dR - dL) / vals.trackingWidth;

    double lsin = sin(pose[2] + 0.5 * dTheta);
    double lcos = cos(pose[2] + 0.5 * dTheta);
    double cOffset = 2 * sin(0.5 * dTheta);

    double A_r = std::abs(dL) > std::abs(dR) ? dL / dTheta + vals.trackingWidth / 2 : dR / dTheta - vals.trackingWidth / 2;
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

const std::array<double,6> &OdomInput::get() const {
  return pose;
}
