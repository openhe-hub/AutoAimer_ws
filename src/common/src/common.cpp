#include "./common/common.hpp"

#include <string>
#include <vector>

#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <opencv2/imgcodecs.hpp>

#include "UltraMultiThread/umt.hpp"

namespace py = pybind11;

auto cpp_str_vec_to_py_list(const std::vector<std::string>& data) -> py::list {
  py::list list;
  for (const auto& value : data) {
    list.append(value);
  }
  return list;
}

py::array_t<uint8_t> cvMat2npArray(const cv::Mat& mat) {
  py::array_t<uint8_t> array({mat.rows, mat.cols, mat.channels()});
  cv::Mat ref_mat(mat.rows, mat.cols, CV_8UC(mat.channels()),
                  array.mutable_data());
  mat.copyTo(ref_mat);
  return array;
}

cv::Mat npArray2cvMat(const py::array_t<uint8_t>& array) {
  cv::Mat mat;
  return mat;
}

UMT_EXPORT_MESSAGE_ALIAS(cvMat, cv::Mat, c) {
  c.def(py::init<cv::Mat>());
  c.def(py::init(&npArray2cvMat));
  c.def("get_nparray", cvMat2npArray);
}

UMT_EXPORT_OBJMANAGER_ALIAS(RangeParam, RangeParam, c) {
  c.def_readwrite("current_value", &RangeParam::current_value);
  c.def_readwrite("min_value", &RangeParam::min_value);
  c.def_readwrite("max_value", &RangeParam::max_value);
  c.def_readwrite("step_value", &RangeParam::step_value);
}

bool Button::is_press_once() {
  if (is_press) {
    is_press = false;
    return true;
  } else {
    return false;
  }
}

void Button::set_press_once() {
  is_press = true;
}

namespace base {
namespace webview_data {}  // namespace webview_data
}  // namespace base

// auto get_group_button_names(const std::string& )

// 自带 name(), find(), create() 的 bind
// 这里 names + find 提供遍历的方法，Button 在 python 那边是动态
// 导入了一整个管理员
UMT_EXPORT_OBJMANAGER_ALIAS(Button, Button, c) {
  c.def(py::init<>());
  // 对于被管理的单个实例可调用
  c.def("is_press_once", &Button::is_press_once);
  c.def("set_press_once", &Button::set_press_once);
}

UMT_EXPORT_OBJMANAGER_ALIAS(WebviewDataPage, ::base::webview_data::Page, c) {
  c.def(py::init<>());
  c.def("get_dict", &::base::webview_data::Page::get);
}

UMT_EXPORT_OBJMANAGER_ALIAS(CheckBox, CheckBox, c) {
  c.def(py::init<>());
  c.def_readwrite("checked", &CheckBox::checked);
}
