#ifndef AIMER_PARAMETER_HPP
#define AIMER_PARAMETER_HPP

#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <thread>

#include <pybind11/numpy.h>
#include <opencv2/core/core.hpp>

#include "UltraMultiThread/include/umt/umt.hpp"

namespace aimer {
namespace param {

struct Integer {
  int value;
};

struct Double {
  double value;
};

std::shared_ptr<aimer::param::Integer> create_int_obj(const std::string& str);

std::shared_ptr<aimer::param::Integer> find_int_obj(const std::string& str);

int find_int_param(const std::string& str);

std::shared_ptr<aimer::param::Double> create_double_obj(const std::string& str);

std::shared_ptr<aimer::param::Double> find_double_obj(const std::string& str);

double find_double_param(const std::string& str);

class PredictorParameter {
 public:
  void load();

  void update();

 private:
  std::set<std::pair<std::string, int>> param_int_info;
  std::set<std::pair<std::string, double>> param_double_info;
  std::set<std::shared_ptr<aimer::param::Integer>> param_int;
  std::set<std::shared_ptr<aimer::param::Double>> param_double;
};

void parameter_run();

}  // namespace param
}  // namespace aimer

#endif
