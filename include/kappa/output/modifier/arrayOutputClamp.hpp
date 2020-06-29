#pragma once

#include "kappa/output/abstractOutput.hpp"
#include <array>
#include <algorithm>
#include <memory>


namespace kappa {

/**
 * Note: requires definition of the operators < > / and *=
 * for T values
 */

template <typename T, std::size_t N>
class ArrayOutputClamp : public AbstractOutput<std::array<T,N>> {
public:
  ArrayOutputClamp(T imin, T imax, std::shared_ptr<AbstractOutput<std::array<T,N>>> ioutput):
    output(ioutput), min(imin), max(imax) {}

  virtual void set(const std::array<T,N> &iTarget) override {
    T maxVal = *std::max_element(iTarget.begin(), iTarget.end());
    T minVal = *std::min_element(iTarget.begin(), iTarget.end());

    if(max < maxVal) {
      if(min > minVal) {
        output->set(scaleArray(iTarget, target, (max / maxVal) < (min / minVal) ? (max / maxVal) : (min / minVal)));
      } else {
        output->set(scaleArray(iTarget, target, max / maxVal));
      }
    } else {
      if(min > minVal) {
        output->set(scaleArray(iTarget, target, min / minVal));
      } else {
        output->set(iTarget);
      }
    }
  }

  std::shared_ptr<AbstractOutput<std::array<T,N>>> getOutput() const {
    return output;
  }

protected:
  std::shared_ptr<AbstractOutput<std::array<T,N>>> output{nullptr};
  std::array<T,N> target;
  T min{0};
  T max{0};

  static std::array<T,N> &scaleArray(const std::array<T,N> &arr, std::array<T,N> &target, T scalar) {
    for(std::size_t i = 0; i < N; i++){
      target[i] = arr[i] * scalar;
    }

    return target;
  }
};

extern template class ArrayOutputClamp<double, 2>;
extern template class ArrayOutputClamp<double, 4>;

}
