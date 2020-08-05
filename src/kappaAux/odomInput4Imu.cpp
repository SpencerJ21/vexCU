#include "kappaAux/odomInput4Imu.hpp"

OdomInput4Imu::OdomInput4Imu(OdomVals &&ivals,
              std::unique_ptr<okapi::Filter> ivelFilter,
              std::unique_ptr<okapi::Filter> istfVelFilter,
              std::unique_ptr<okapi::Filter> iangVelFilter,
              std::shared_ptr<kappa::AbstractInput<std::array<double,5>>> iinput):
              input(iinput), vals(ivals), velFilter(std::move(ivelFilter)),
              stfVelFilter(std::move(istfVelFilter)),
              angVelFilter(std::move(iangVelFilter))
              {}

const std::array<double,6> &OdomInput4Imu::step() {
  const std::array<double,5> &in = input->get();

  double dL     = (in[0] - lastIn[0]);
  double dB     = (in[1] - lastIn[1]);
  double dR     = (in[2] - lastIn[2]);
  double dF     = (in[3] - lastIn[3]);
  double dTheta = (in[4] - lastIn[4]);

  lastIn[0] = in[0];
  lastIn[1] = in[1];
  lastIn[2] = in[2];
  lastIn[3] = in[3];
  lastIn[4] = in[4];

  if(dTheta == 0){
    double lsin = sin(pose[2] * M_PI/180.0);
    double lcos = cos(pose[2] * M_PI/180.0);

    double dV  = (dL + dR) / 2.0;
    double dSV = (dB + dF) / 2.0;

    pose[0] += lcos * dV -
               lsin * dSV;
    pose[1] += lcos * dSV +
               lsin * dV;

    pose[3] = velFilter->   filter(dV  / vals.timestep);
    pose[4] = stfVelFilter->filter(dSV / vals.timestep);
    pose[5] = angVelFilter->filter(0);
  }else{
    double lsin = sin((pose[2] * M_PI/180.0) + (dTheta * M_PI/360.0));
    double lcos = cos((pose[2] * M_PI/180.0) + (dTheta * M_PI/360.0));
    double cOffset = 2 * sin(dTheta / 2);

    double A_r = std::abs(dL) > std::abs(dR) ? dL / dTheta + vals.rlTrackingWidth / 2 : dR / dTheta - vals.rlTrackingWidth / 2;
    double S_r = std::abs(dB) > std::abs(dF) ? dB / dTheta + vals.fbTrackingWidth / 2 : dF / dTheta - vals.fbTrackingWidth / 2;

    pose[0] += cOffset * (lcos * A_r - lsin * S_r);
    pose[1] += cOffset * (lcos * S_r + lsin * A_r);
    pose[2] += dTheta;
    pose[3] = velFilter->   filter(A_r * dTheta / vals.timestep);
    pose[4] = stfVelFilter->filter(S_r * dTheta / vals.timestep);
    pose[5] = angVelFilter->filter(      dTheta / vals.timestep);
  }

  return pose;
}

const std::array<double,6> &OdomInput4Imu::get() {
  return pose;
}
