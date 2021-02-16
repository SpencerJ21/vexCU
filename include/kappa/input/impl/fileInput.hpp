#pragma once

#include "kappa/input/simpleInput.hpp"
#include <fstream>
#include <array>
#include <string>
#include <iostream>

namespace kappa {

template <std::size_t N>
class FileInput : public SimpleInput<std::array<double,N>> {
public:

  /**
   * Opens a CSV file of doubles. For each iteration, it takes one row of values
   * and passes it as an array of values
   *
   * @param filename path to file (make sure it begins with /usd/)
   */
  FileInput(const std::string &filename):
    file(filename){}

  ~FileInput() {
    file.close();
  }

  /**
   * gets values from the file
   *
   * @return values
   */
  virtual const std::array<double,N> &get() override {
    if(!finished) {
      std::getline(file, line, '\n');

      index = 0;
      step = 0;

      try{
        for(std::size_t i = 0; i < N; i++){
          value[i] = std::stod(line.substr(index), &step);
          index += (step + 1);
        }
      }catch(...){ // Will catch if there is no remaining valid doubles - assumes end of file
        finished = true;
      }
    }

    return value;
  }

  /**
   * gets previous values, without getting new values from the file
   *
   * @returns values
   */
  virtual const std::array<double,N> &getValue() const override {
    return value;
  }

protected:
  std::ifstream file;
  std::string line;
  std::array<double,N> value;
  std::size_t index, step;
  bool finished{false};
};

extern template class FileInput<1>;
extern template class FileInput<2>;
extern template class FileInput<3>;
extern template class FileInput<4>;
extern template class FileInput<5>;
extern template class FileInput<6>;
extern template class FileInput<7>;

}
