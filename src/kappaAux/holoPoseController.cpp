#include "kappaAux/holoPoseController.hpp"

HoloPoseController::HoloPoseController(std::unique_ptr<kappa::PidController> ilinearController,
                                       std::unique_ptr<kappa::PidController> iangularController):
                                       linearController(std::move(ilinearController)),
                                       angularController(std::move(iangularController)),
                                       stopping(true){ reset(); }

void HoloPoseController::setTarget(const Pose &itarget) {
  target = itarget;

  linearController->setTarget(0); // drive distance (to target) to 0
  angularController->setTarget(target.theta); //drive heading to target theta
}

void HoloPoseController::setOutputLimits(std::tuple<double,double,double> imin, std::tuple<double,double,double> imax) {
  outputMin = imin;
  outputMax = imax;

  linearController->setOutputLimits(std::get<0>(imin), std::get<0>(imax));
  angularController->setOutputLimits(std::get<2>(imin), std::get<2>(imax));
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

  // Error vector in global coords
  double dy = target.y - ireading.y;
  double dx = target.x - ireading.x;

  // Distance error
  distance = sqrt(dy * dy + dx * dx);

  return {
    std::clamp(linearController->step(-distance), std::get<0>(outputMin), std::get<0>(outputMax)),      // Apply PID controller to distance and set to linear speed
    atan2(dy,dx) - ireading.theta,                                                                      // Heading to target point in local coordinates
    std::clamp(angularController->step(ireading.theta), std::get<2>(outputMin), std::get<2>(outputMax)) // Apply PID controller to angle error and set to angular speed
  };
}

void HoloPoseController::stop(){
  output = {0, 0, 0};
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
