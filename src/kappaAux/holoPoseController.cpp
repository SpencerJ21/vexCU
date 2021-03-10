#include "kappaAux/holoPoseController.hpp"

HoloPoseController::HoloPoseController(std::unique_ptr<kappa::PidController> ilinearController,
                                       std::unique_ptr<kappa::PidController> iangularController):
                                       linearController(std::move(ilinearController)),
                                       angularController(std::move(iangularController)),
                                       stopping(true){ reset(); }

void HoloPoseController::setTarget(const Pose &itarget) {
  target = itarget;
}

void HoloPoseController::setOutputLimits(std::tuple<double,double,double> imin, std::tuple<double,double,double> imax) {
  outputMin = imin;
  outputMax = imax;
}

std::tuple<double,double,double> HoloPoseController::getMinOutput() const {
  return outputMin;
}

std::tuple<double,double,double> HoloPoseController::getMaxOutput() const {
  return outputMax;
}

std::tuple<double,double,double> HoloPoseController::step(Pose ireading){
  if(disabled){
    return {0,0,0};
  }

  double dy = target.y - ireading.y;
  double dx = target.x - ireading.x;

  distance = sqrt(dy * dy + dx * dx);

  return {
    stopping ? std::clamp(linearController->step(distance), std::get<0>(outputMin), std::get<0>(outputMax)) : std::get<0>(outputMax),
    atan2(dy,dx) - ireading.theta,
    std::clamp(angularController->step(ireading.theta), std::get<2>(outputMin), std::get<2>(outputMax))
  };
}

bool HoloPoseController::isSettled(){
  if(stopping){
    return linearController->isSettled() && angularController->isSettled();
  }else{
    return distance < 3 && angularController->isSettled();
  }
}

void HoloPoseController::reset(){
  linearController->reset();
  angularController->reset();
  target = {0,0,0,0,0,0};
}

void HoloPoseController::disable(bool iisDisabled) {
  disabled = iisDisabled;
}

void HoloPoseController::setStopping(bool istopping){
  stopping = istopping;
}
