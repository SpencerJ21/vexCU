#include "kappaAux/odometry.hpp"

// IF YOU TAKE THIS CODE AND COPY PASTE AND USE IT INSTEAD OF LEARNING/MAKING YOUR
// OWN ODOMETRY CODE YOU ARE BAD AND SHOULD FEEL BAD. GO READ THE PILONS DOCUMENT
// AND CREATE YOUR OWN CODE BASED OFF IT

Odom3EncImu::Odom3EncImu(
              double irearOffset,
              std::unique_ptr<okapi::Filter> itanVelFilter,
              std::unique_ptr<okapi::Filter> istfVelFilter,
              std::unique_ptr<okapi::Filter> iangVelFilter,
              std::shared_ptr<kappa::AbstractInput<std::array<double,4>>> iinput):
              input(iinput), rearOffset(irearOffset),
              tanVelFilter(std::move(itanVelFilter)),
              stfVelFilter(std::move(istfVelFilter)),
              angVelFilter(std::move(iangVelFilter))
  {
    lastIn[4] = pros::millis();
  }

const Pose &Odom3EncImu::step() {
  // Essentially the same as pilons but apply small angle approx to simplify

  const std::array<double,4> &in = input->get();

  double dT = 0.001 * (pros::millis() - lastIn[4]);


  if(dT == 0) return pose;

  lastIn[4] = pros::millis();

  double dL     =  in[0] - lastIn[0];
  double dM     =  in[1] - lastIn[1];
  double dR     =  in[2] - lastIn[2];
  double dTheta = (in[3] - lastIn[3]) * M_PI / 180;

  lastIn[0] = in[0];
  lastIn[1] = in[1];
  lastIn[2] = in[2];
  lastIn[3] = in[3];

  double dS = (dR + dL) / 2.0;
  double dN = dM + dTheta * rearOffset;

  double lsin = sin(pose.theta + 0.5 * dTheta);
  double lcos = cos(pose.theta + 0.5 * dTheta);

  pose.x     += lcos * dS -
                lsin * dN;
  pose.y     += lcos * dN +
                lsin * dS;
  pose.theta += dTheta;

  pose.tanVelocity = tanVelFilter->filter(dS     / dT);
  pose.latVelocity = stfVelFilter->filter(dN     / dT);
  pose.angVelocity = angVelFilter->filter(dTheta / dT);

  return pose;
}

const Pose &Odom3EncImu::get() {
  return pose;
}
