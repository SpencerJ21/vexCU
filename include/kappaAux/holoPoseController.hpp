#pragma once
#include "main.h"
#include "kappaAux/odometry.hpp"

class HoloPoseController : public kappa::AbstractController<Pose, Pose, std::tuple<double,double,double>> {
public:

  HoloPoseController(std::unique_ptr<kappa::PidController> ilinearController,
                     std::unique_ptr<kappa::PidController> iangularController);

  virtual void setTarget(const Pose &itarget) override;

  virtual void setOutputLimits(std::tuple<double,double,double> imin, std::tuple<double,double,double> imax);

  virtual std::tuple<double,double,double> getMinOutput() const;

  virtual std::tuple<double,double,double> getMaxOutput() const;

  virtual std::tuple<double,double,double> step(Pose ireading) override;

  virtual bool isSettled() override;

  virtual void reset() override;

  virtual void disable(bool iisDisabled) override;

protected:

  std::unique_ptr<kappa::PidController> linearController;
  std::unique_ptr<kappa::PidController> angularController;

  std::tuple<double,double,double> outputMin{-DBL_MAX, -DBL_MAX, -DBL_MAX};
  std::tuple<double,double,double> outputMax{ DBL_MAX,  DBL_MAX,  DBL_MAX};

};
