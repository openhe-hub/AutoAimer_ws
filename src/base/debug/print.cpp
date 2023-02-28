#include "debug/print.hpp"

#include <fmt/color.h>
#include <fmt/format.h>

namespace base {
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

}  // namespace base
