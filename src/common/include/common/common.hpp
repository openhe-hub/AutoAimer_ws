#ifndef _CVRM2021_WEBKITS_HPP_
#define _CVRM2021_WEBKITS_HPP_

#include <iostream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "UltraMultiThread/ObjManager.hpp"

namespace py = pybind11;

// auto cpp_str_vec_to_py_list(const std::vector<std::string>& data) ->
// py::list;

struct RangeParam {
  double current_value = 0;
  double min_value = 0;
  double max_value = 255;
  double step_value = 1;
};

class Button {
 public:
  bool is_press_once();

  void set_press_once();

 private:
  bool is_press = false;
};

namespace base {
namespace webview_data {

namespace umt = ::umt;

// 类型特征萃取器 type traits
// 拥有 get() 函数才返回
template <typename T>
struct has_get_function {
  template <typename U>
  static auto test(U* p) -> decltype(p->get(), std::true_type{});

  template <typename U>
  static auto test(...) -> std::false_type;

  static constexpr bool value = decltype(test<T>(nullptr))::value;
};

template <typename T>
using EnableIfHasGetFunction = std::enable_if_t<has_get_function<T>::value>;

// get() 负责建立一层，SubType 也必须有 get()
template <typename SubType, typename = EnableIfHasGetFunction<SubType>>
class Tree {
 public:
  auto sub(const std::string& name) -> SubType& {
    if (this->sub_map.count(name) == 0u) {
      this->sub_cnt += 1;
      this->sub_map.insert({name, {this->sub_cnt, {}}});
    }
    return this->sub_map.at(name).second;
  }

  auto get() -> py::dict {
    std::vector<std::pair<int, std::string>> sorted;
    for (auto& [name, wrapped] : this->sub_map) {
      sorted.push_back({wrapped.first, name});
    }
    std::sort(sorted.begin(), sorted.end());
    py::dict dict;
    for (auto& [pri, name] : sorted) {
      auto it = this->sub_map.find(name);
      dict[it->first.c_str()] = it->second.second.get();
    }
    return dict;
  }

 private:
  int sub_cnt = 0;
  // 第二成员的第一成员表示插入顺序
  std::map<std::string, std::pair<int, SubType>> sub_map;
};

class Entry {
 public:
  auto get() -> std::string& { return this->content_str; }

 private:
  std::string content_str;
};

using Group = Tree<Entry, EnableIfHasGetFunction<Entry>>;
using Page = Tree<Group, EnableIfHasGetFunction<Group>>;
// using Page = Tree<Group>;

// class Group {
//  public:
//   auto entry(const std::string& name) -> webview_data::Entry& {
//     if (this->entry_map.count(name) == 0u) {
//       this->entry_map.insert({name, {}});
//     }
//     return this->entry_map.at(name);
//   }

//   auto get_dict() -> py::dict;

//  private:
//   int entry_cnt;
//   std::unordered_map<std::string, webview_data::Entry> entry_map;
// };

// class Page {
//  public:
//   auto group(const std::string& name) -> webview_data::Group& {
//     if (this->group_map.count(name) == 0u) {
//       this->group_cnt += 1;
//       this->group_map.insert({name, {this->group_cnt, {}}});
//     }
//     return this->group_map.at(name).second;
//   }

//   auto get_dict() -> py::dict;

//  private:
//   int group_cnt;
//   // 第二成员的第一成员表示优先级
//   std::map<std::string, std::pair<int, webview_data::Group>> group_map;
// };

}  // namespace webview_data
}  // namespace base

struct CheckBox {
  bool checked;
};

#endif /* _CVRM2021_WEBKITS_HPP_ */
