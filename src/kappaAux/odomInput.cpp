#include "kappaAux/odomInput.hpp"

OdomInput::OdomInput(OdomVals &&ivals,
          std::unique_ptr<okapi::Filter> ivelFilter,
          std::unique_ptr<okapi::Filter> istfVelFilter,
          std::unique_ptr<okapi::Filter> iangVelFilter,
          std::shared_ptr<kappa::AbstractInput<std::array<double,3>>> iinput):
          input(iinput), vals(ivals), velFilter(std::move(ivelFilter)),
          stfVelFilter(std::move(istfVelFilter)),
          angVelFilter(std::move(iangVelFilter))
          {}

const std::array<double,6> &OdomInput::step() {
  const std::array<double,3> &in = input->get();

  double dL = (in[0] - lastIn[0]);
  double dM = (in[1] - lastIn[1]);
  double dR = (in[2] - lastIn[2]);

  lastIn[0] = in[0];
  lastIn[1] = in[1];
  lastIn[2] = in[2];

  if(dL == dR){
    double lsin = sin(pose[2] * M_PI/180.0);
    double lcos = cos(pose[2] * M_PI/180.0);

    pose[0] += lcos * dR -
               lsin * dM;
    pose[1] += lcos * dM +
               lsin * dR;

    pose[3] = velFilter->   filter(dR / vals.timestep);
    pose[4] = stfVelFilter->filter(dM / vals.timestep);
    pose[5] = angVelFilter->filter(0);
  }else{
    double dTheta = (dR - dL) / vals.trackingWidth;

    double lsin = sin((pose[2] * M_PI/180.0) + (dTheta * M_PI/360.0));
    double lcos = cos((pose[2] * M_PI/180.0) + (dTheta * M_PI/360.0));
    double cOffset = 2 * sin(dTheta / 2);

    double dV = (dL + dR) / 2.0;
    double M_r = dM / dTheta + vals.rearTrackingDist;

    double dlX = cOffset * dV / dTheta;
    double dlY = cOffset * M_r;

    pose[0] += lcos * dlX -
               lsin * dlY;
    pose[1] += lcos * dlY +
               lsin * dlX;
    pose[2] += dTheta;
    pose[3] = velFilter->   filter(dV           / vals.timestep);
    pose[4] = stfVelFilter->filter(M_r * dTheta / vals.timestep);
    pose[5] = angVelFilter->filter(dTheta       / vals.timestep);
  }

  return pose;
}

const std::array<double,6> &OdomInput::get() {
  return pose;
}
