#include "kappaAux/odomInputChenImu.hpp"

OdomInputChenImu::OdomInputChenImu(OdomVals &&ivals,
           std::unique_ptr<okapi::Filter> ivelFilter,
           std::unique_ptr<okapi::Filter> istfVelFilter,
           std::unique_ptr<okapi::Filter> iangVelFilter,
           std::shared_ptr<kappa::AbstractOutput<std::array<double,6>>> ioutput):
           output(ioutput), vals(ivals), velFilter(std::move(ivelFilter)),
           stfVelFilter(std::move(istfVelFilter)),
           angVelFilter(std::move(iangVelFilter))
  {
   lastIn[5] = pros::millis();
  }

void OdomInputChenImu::set(const std::array<double,5> &input) {

  double dL = input[0] - lastIn[0];
  double dB = input[2] - lastIn[1];
  double dR = input[1] - lastIn[2];
  double dF = input[3] - lastIn[3];
  double dT = pros::millis() - lastIn[5];

  if(dT == 0) return;

  lastIn[5] = pros::millis();
  lastIn[0] = input[0];
  lastIn[1] = input[2];
  lastIn[2] = input[1];
  lastIn[3] = input[3];

  double dTheta;

  if(input[4] != lastIn[4]){
    dTheta = input[4] * M_PI / 180 - pose[2];
    lastIn[4] = input[4];
  }else{
    dTheta = (dR - dL) / (2 * vals.rlTrackingWidth) +
             (dF - dB) / (2 * vals.fbTrackingWidth);
  }

  double dV = (dR + dL) / 2.0;
  double dSV = (dF + dB) / 2.0;

  double lsin = sin((pose[2]) + 0.5 * dTheta);
  double lcos = cos((pose[2]) + 0.5 * dTheta);

  pose[0] += lcos * dV -
             lsin * dSV;
  pose[1] += lcos * dSV +
             lsin * dV;
  pose[2] += dTheta;

  pose[3] = velFilter->   filter(dV  / dT);
  pose[4] = stfVelFilter->filter(dSV / dT);
  pose[5] = angVelFilter->filter(dTheta / dT);

  output->set(pose);
}

const std::array<double,6> &OdomInputChenImu::get() const {
  return pose;
}
