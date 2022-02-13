#pragma once

#include <cppystruct/cppystruct.h>

#include <cmath>
#include <cstdint>
#include <string_view>

#include "mycobot/bytearray.hpp"

namespace mycobot {

int8_t decode_int8(bytearray const& data);
int16_t decode_int16(bytearray const& data);
bytearray encode(int8_t data);
bytearray encode(int16_t data);

constexpr int16_t angle2int(double angle) {
  return static_cast<int16_t>(std::round(angle * 100.0));
}

constexpr int16_t coord2int(double coord) {
  return static_cast<int16_t>(std::round(coord * 10.0));
}

constexpr double int2angle(int16_t data) { return data / 100.0; }

constexpr double int2coord(int16_t data) { return data / 10.0; }

}  // namespace mycobot
