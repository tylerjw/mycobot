#pragma once
#include <array>
#include <cstdint>
#include <string>

namespace mycobot {

using bytearray = std::basic_string<int8_t>;

template <size_t N>
bytearray to_bytearray(std::array<char, N> const& arr) {
  return bytearray(arr.begin(), arr.end());
}

bytearray to_bytearray(std::string const& str);

}  // namespace mycobot
