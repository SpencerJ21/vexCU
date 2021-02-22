#include "kappaAux/odometry.hpp"

Odom3EncImu::Odom3EncImu(
              std::unique_ptr<okapi::Filter> itanVelFilter,
              std::unique_ptr<okapi::Filter> istfVelFilter,
              std::unique_ptr<okapi::Filter> iangVelFilter,
              std::shared_ptr<kappa::AbstractInput<std::array<double,4>>> iinput):
              input(iinput), tanVelFilter(std::move(itanVelFilter)),
              stfVelFilter(std::move(istfVelFilter)),
              angVelFilter(std::move(iangVelFilter))
  {
    lastIn[4] = pros::millis();
  }

const Pose &Odom3EncImu::step() {
  const std::array<double,4> &in = input->get();

  double dT = 0.001 * (pros::millis() - lastIn[4]);


  if(dT == 0) return pose;

  lastIn[4] = pros::millis();

  double dL     =  in[0] - lastIn[0];
  double dSL    =  in[1] - lastIn[1];
  double dR     =  in[2] - lastIn[2];
  double dTheta = (in[3] - lastIn[3]) * M_PI / 180;

  lastIn[0] = in[0];
  lastIn[1] = in[1];
  lastIn[2] = in[2];
  lastIn[3] = in[3];

  double dS = (dR + dL) / 2.0;

  double lsin = sin(pose.theta + 0.5 * dTheta);
  double lcos = cos(pose.theta + 0.5 * dTheta);

  pose.x     += lcos * dS -
                lsin * dSL;
  pose.y     += lcos * dSL +
                lsin * dS;
  pose.theta += dTheta;

  pose.tanVelocity = tanVelFilter->filter(dS     / dT);
  pose.latVelocity = stfVelFilter->filter(dSL    / dT);
  pose.angVelocity = angVelFilter->filter(dTheta / dT);

  return pose;
}

const Pose &Odom3EncImu::get() {
  return pose;
}
