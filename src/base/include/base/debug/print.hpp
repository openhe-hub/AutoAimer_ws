#ifndef BASE_DEBUG_PRINT_HPP
#define BASE_DEBUG_PRINT_HPP

// ::fmt
#include <fmt/color.h>
#include <unordered_map>

namespace base {
namespace fmt = ::fmt;

enum class PrintMode { Info, Warning, Error, Panic };

// extern const std::unordered_map<base::PrintMode, fmt::color> PRINT_COLOR;

// extern const std::unordered_map<base::PrintMode, std::string> PRINT_PREFIX;

const std::unordered_map<base::PrintMode, fmt::color> PRINT_COLOR = {
    {PrintMode::Info, fmt::color::cornflower_blue},
    {PrintMode::Warning, fmt::color::orange},
    {PrintMode::Error, fmt::color::red},
    {PrintMode::Panic, fmt::color::red}};

const std::unordered_map<base::PrintMode, std::string> PRINT_PREFIX = {
    {PrintMode::Info, "[INFO]"},
    {PrintMode::Warning, "[WARNING]"},
    {PrintMode::Error, "[ERROR]"},
    {PrintMode::Panic, "[PANIC]"}};

template <typename... T>
auto print(const base::PrintMode& mode,
           const std::string& node_name,
           const std::string& content,
           T&&... args) {
  fmt::print(fmt::fg(base::PRINT_COLOR.at(mode)),
             base::PRINT_PREFIX.at(mode) + " @" + node_name + ": " + content,
             args...);
}
}  // namespace base

#endif /* BASE_DEBUG_PRINT_HPP */
