#pragma once
#include "kappa/api.hpp"

class OdomInput : public kappa::ComputationalInput<std::array<double,6>> {
public:
  struct OdomVals {
    double trackingWidth;
    double rearTrackingDist;
    double timestep;
  };

  OdomInput(OdomVals &&ivals,
            std::unique_ptr<okapi::Filter> ivelFilter,
            std::unique_ptr<okapi::Filter> istfVelFilter,
            std::unique_ptr<okapi::Filter> iangVelFilter,
            std::shared_ptr<kappa::AbstractInput<std::array<double,3>>> iinput); // left, middle, right (positive dir front;left)

  virtual const std::array<double,6> &step();

  virtual const std::array<double,6> &get();

protected:
  std::shared_ptr<kappa::AbstractInput<std::array<double,3>>> input;
  OdomVals vals;
  std::unique_ptr<okapi::Filter> velFilter;
  std::unique_ptr<okapi::Filter> stfVelFilter;
  std::unique_ptr<okapi::Filter> angVelFilter;

  std::array<double,6> pose{0,0,0,0,0,0};

  std::array<double,3> lastIn{0,0,0};
};
