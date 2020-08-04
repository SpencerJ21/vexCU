#include "kappaAux/odomInput4.hpp"

OdomInput4::OdomInput4(OdomVals &&ivals,
           std::unique_ptr<okapi::Filter> ivelFilter,
           std::unique_ptr<okapi::Filter> istfVelFilter,
           std::unique_ptr<okapi::Filter> iangVelFilter,
           std::shared_ptr<kappa::AbstractInput<std::array<double,4>>> iinput):
           input(iinput), vals(ivals), velFilter(std::move(ivelFilter)),
           stfVelFilter(std::move(istfVelFilter)),
           angVelFilter(std::move(iangVelFilter))
           {}

const std::array<double,6> &OdomInput4::step() {
  const std::array<double,4> &in = input->get();

  double dL = (in[0] - lastIn[0]);
  double dB = (in[1] - lastIn[1]);
  double dR = (in[2] - lastIn[2]);
  double dF = (in[3] - lastIn[3]);

  lastIn[0] = in[0];
  lastIn[1] = in[1];
  lastIn[2] = in[2];
  lastIn[3] = in[3];

  if(dL + dB == dR + dF){
    double lsin = sin(pose[2] * M_PI/180.0);
    double lcos = cos(pose[2] * M_PI/180.0);

    pose[0] += (lcos * (dL + dR) -
                lsin * (dB + dF)) * 0.5;
    pose[1] += (lcos * (dB + dF) +
                lsin * (dL + dR)) * 0.5;

    pose[3] = velFilter->   filter((dL + dR) / (2 * vals.timestep));
    pose[4] = stfVelFilter->filter((dB + dF) / (2 * vals.timestep));
    pose[5] = angVelFilter->filter(0);
  }else{
    double dTheta = (dR - dL) / (2 * vals.rlTrackingWidth) +
                    (dF - dB) / (2 * vals.fbTrackingWidth);

    double lsin = sin((pose[2] * M_PI/180.0) + (dTheta * M_PI/360.0));
    double lcos = cos((pose[2] * M_PI/180.0) + (dTheta * M_PI/360.0));
    double cOffset = 2 * sin(dTheta / 2);

    double dlX = cOffset * (dL + dR) / (2 * dTheta);
    double dlY = cOffset * (dB + dF) / (2 * dTheta);

    pose[0] += lcos * dlX -
               lsin * dlY;
    pose[1] += lcos * dlY +
               lsin * dlX;
    pose[2] += dTheta;
    pose[3] = velFilter->   filter(dlY    / vals.timestep);
    pose[4] = stfVelFilter->filter(dlX    / vals.timestep);
    pose[5] = angVelFilter->filter(dTheta / vals.timestep);
  }

  return pose;
}

const std::array<double,6> &OdomInput4::get() {
  return pose;
}
