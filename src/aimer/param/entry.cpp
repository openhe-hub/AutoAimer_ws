#include <iostream>

#include "UltraMultiThread/include/umt/umt.hpp"
#include "aimer/param/parameter.hpp"

namespace aimer {
void background_parameter_run() {
  std::thread([]() { aimer::param::parameter_run(); }).detach();
}
}  // namespace aimer

PYBIND11_EMBEDDED_MODULE(aimer_param, m) {
  // namespace py = pybind11;
  m.def("background_parameter_run", aimer::background_parameter_run);
}
