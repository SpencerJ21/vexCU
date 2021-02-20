#pragma once

#include "kappa/output/impl/xdriveChassis.hpp"
#include <tuple>
#include <memory>


class HolonomicSlew : public kappa::AbstractOutput<std::tuple<double,double,double>> {
public:

  /**
   * Applies a slew controller to the target velocity of a holonomic chassis in polar form
   *
   * @param ispdSlewStep the maximum amount the speed can change in one iteration
   * @param idirSlewStep the maximum amount the direction can change in one
   *    iteration per unit speed (should be large)
   * @param ichassis XDrive chassis
   */
  HolonomicSlew(double ispdSlewStep, double idirSlewStep, std::shared_ptr<kappa::XDriveChassis> ichassis);

  /**
   * Using a polar target signal of (velocity, direction, rotation),
   * calculates target values for each motor and sets their respective targets
   *
   * @param itarget target values
   */
  virtual void set(const std::tuple<double,double,double> &itarget) override;

  /**
   * Gets output
   *
   * @return output
   */
  std::shared_ptr<kappa::XDriveChassis> getOutput() const;

protected:
  std::shared_ptr<kappa::XDriveChassis> chassis{nullptr};
  double spdSlewStep{0};
  double dirSlewStep{0};

  std::tuple<double,double,double> out{0,0,0};
};
