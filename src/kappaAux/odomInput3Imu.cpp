#include "kappaAux/odomInput3Imu.hpp"

OdomInput3Imu::OdomInput3Imu(OdomVals &&ivals,
              std::unique_ptr<okapi::Filter> ivelFilter,
              std::unique_ptr<okapi::Filter> istfVelFilter,
              std::unique_ptr<okapi::Filter> iangVelFilter,
              std::shared_ptr<kappa::AbstractInput<std::array<double,4>>> iinput):
              input(iinput), vals(ivals), velFilter(std::move(ivelFilter)),
              stfVelFilter(std::move(istfVelFilter)),
              angVelFilter(std::move(iangVelFilter))
              {}

const std::array<double,6> &OdomInput3Imu::step() {
  const std::array<double,4> &in = input->get();

  double dL     = (in[0] - lastIn[0]);
  double dM     = (in[1] - lastIn[1]);
  double dR     = (in[2] - lastIn[2]);
  double dTheta = (in[3] - lastIn[3]);

  lastIn[0] = in[0];
  lastIn[1] = in[1];
  lastIn[2] = in[2];
  lastIn[3] = in[3];

  if(dTheta == 0){
    double lsin = sin(pose[2] * M_PI/180.0);
    double lcos = cos(pose[2] * M_PI/180.0);

    double dV = (dL + dR) / 2;

    pose[0] += lcos * dV -
               lsin * dM;
    pose[1] += lcos * dM +
               lsin * dV;

    pose[3] = velFilter->   filter(dV / vals.timestep);
    pose[4] = stfVelFilter->filter(dM / vals.timestep);
    pose[5] = angVelFilter->filter(0);
  }else{
    double lsin = sin((pose[2] * M_PI/180.0) + (dTheta * M_PI/360.0));
    double lcos = cos((pose[2] * M_PI/180.0) + (dTheta * M_PI/360.0));
    double cOffset = 2 * sin(dTheta / 2);

    double A_r = std::abs(dL) > std::abs(dR) ? dL / dTheta + vals.trackingWidth / 2 : dR / dTheta - vals.trackingWidth / 2;
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

const std::array<double,6> &OdomInput3Imu::get() {
  return pose;
}
