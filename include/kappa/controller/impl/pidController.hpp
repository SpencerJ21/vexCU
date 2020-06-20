#pragma once

#include "kappa/controller/abstractController.hpp"
#include "okapi/api/control/util/settledUtil.hpp"
#include "okapi/api/filter/filter.hpp"
#include "okapi/api/filter/passthroughFilter.hpp"
#include "okapi/impl/util/timer.hpp"
#include <memory>

namespace kappa {

class PidController : public AbstractController<double, double, double> {
public:

  struct Gains {
    double kP;
    double kI;
    double kD;
    double kF;
  };

  PidController(Gains igains,
                std::unique_ptr<okapi::SettledUtil> isettledUtil = std::make_unique<okapi::SettledUtil>(std::unique_ptr<okapi::Timer>()),
                std::unique_ptr<okapi::Filter> iderivativeFilter = std::make_unique<okapi::PassthroughFilter>());

  virtual void setTarget(const double &itarget) override;

  virtual double step(double ireading) override;

  virtual bool isSettled() override;

  virtual void reset() override;

  virtual void disable(bool iisDisabled) override;

  Gains getGains() const;

  void setGains(Gains igains);

protected:
  Gains gains;
  std::unique_ptr<okapi::SettledUtil> settledUtil{nullptr};
  std::unique_ptr<okapi::Filter> derivativeFilter{nullptr};

  double integral;
  double derivative;
};

}
