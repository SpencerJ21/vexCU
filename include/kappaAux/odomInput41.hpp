#pragma once
#include "kappa/api.hpp"

class OdomInput41 : public kappa::AbstractOutput<std::array<double,5>> {
public:
  struct OdomVals {
    double rlTrackingWidth;
    double fbTrackingWidth;
  };

  OdomInput41(OdomVals &&ivals,
             std::unique_ptr<okapi::Filter> ivelFilter,
             std::unique_ptr<okapi::Filter> istfVelFilter,
             std::unique_ptr<okapi::Filter> iangVelFilter,
             std::shared_ptr<kappa::AbstractOutput<std::array<double,6>>> ioutput); // left, back, right, front (positive dir front;left)

  virtual void set(const std::array<double,5> &input);

  const std::array<double,6> &get() const;

protected:
  std::shared_ptr<kappa::AbstractOutput<std::array<double,6>>> output;
  OdomVals vals;
  std::unique_ptr<okapi::Filter> velFilter;
  std::unique_ptr<okapi::Filter> stfVelFilter;
  std::unique_ptr<okapi::Filter> angVelFilter;

  std::array<double,6> pose{0,0,0,0,0,0};

  std::array<double,5> lastIn{0,0,0,0,0};
};
