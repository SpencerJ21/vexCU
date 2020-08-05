#include "kappaAux/odomInput2Imu.hpp"

OdomInput2Imu::OdomInput2Imu(OdomVals &&ivals,
              std::unique_ptr<okapi::Filter> ivelFilter,
              std::unique_ptr<okapi::Filter> istfVelFilter,
              std::unique_ptr<okapi::Filter> iangVelFilter,
              std::shared_ptr<kappa::AbstractInput<std::array<double,3>>> iinput):
              input(iinput), vals(ivals), velFilter(std::move(ivelFilter)),
              stfVelFilter(std::move(istfVelFilter)),
              angVelFilter(std::move(iangVelFilter))
              {}

const std::array<double,6> &OdomInput2Imu::step() {
  const std::array<double,3> &in = input->get();

  double dL     = (in[0] - lastIn[0]);
  double dM     = (in[1] - lastIn[1]);
  double dTheta = (in[2] - lastIn[2]);

  lastIn[0] = in[0];
  lastIn[1] = in[1];
  lastIn[2] = in[2];

  if(dTheta == 0){
    double lsin = sin(pose[2] * M_PI/180.0);
    double lcos = cos(pose[2] * M_PI/180.0);

    pose[0] += lcos * dL -
               lsin * dM;
    pose[1] += lcos * dM +
               lsin * dL;

    pose[3] = velFilter->   filter(dL / vals.timestep);
    pose[4] = stfVelFilter->filter(dM / vals.timestep);
    pose[5] = angVelFilter->filter(0);
  }else{
    double lsin = sin((pose[2] * M_PI/180.0) + (dTheta * M_PI/360.0));
    double lcos = cos((pose[2] * M_PI/180.0) + (dTheta * M_PI/360.0));
    double cOffset = 2 * sin(dTheta / 2);

    double A_r = dL / dTheta + vals.sideTrackingDist;
    double M_r = dM / dTheta + vals.rearTrackingDist;

    pose[0] += cOffset * (lcos * A_r - lsin * M_r);
    pose[1] += cOffset * (lcos * M_r + lsin * A_r);
    pose[2] += dTheta;
    pose[3] = velFilter->   filter(A_r * dTheta / vals.timestep);
    pose[4] = stfVelFilter->filter(M_r * dTheta / vals.timestep);
    pose[5] = angVelFilter->filter(      dTheta / vals.timestep);
  }

  return pose;
}

const std::array<double,6> &OdomInput2Imu::get() {
  return pose;
}
