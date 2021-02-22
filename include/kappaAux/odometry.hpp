#pragma once
#include "kappa/api.hpp"

struct Pose {
  double x;
  double y;
  double theta;

  double tanVelocity;
  double latVelocity;
  double angVelocity;
};

class Odom3EncImu : public kappa::ComputationalInput<Pose> {
public:
  Odom3EncImu(std::unique_ptr<okapi::Filter> itanVelFilter,
              std::unique_ptr<okapi::Filter> ilatVelFilter,
              std::unique_ptr<okapi::Filter> iangVelFilter,
              std::shared_ptr<kappa::AbstractInput<std::array<double,4>>> iinput); // left, back, right, imu (positive dir front;left;ccw)

  virtual const Pose &step();

  virtual const Pose &get();

protected:
  std::shared_ptr<kappa::AbstractInput<std::array<double,4>>> input;
  std::unique_ptr<okapi::Filter> tanVelFilter;
  std::unique_ptr<okapi::Filter> stfVelFilter;
  std::unique_ptr<okapi::Filter> angVelFilter;

  Pose pose{0,0,0,0,0,0};

  std::array<double,5> lastIn{0,0,0,0,0}; // left, back, right, imu, time
};
