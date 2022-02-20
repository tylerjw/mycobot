#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace mycobot {

int8_t decode_int8(std::string const& data);
int16_t decode_int16(std::string const& data);

std::string encode(char data);
std::string encode(int8_t data);
std::string encode(int16_t data);

template <typename T, typename... Args>
std::string encode(T first, Args... args) {
  auto ret = std::string{}.append(encode(first));
  auto rest = encode(args...);
  return ret.append(rest);
}

constexpr int16_t angle2int(double angle) {
  return static_cast<int16_t>(std::round(angle * 100.0));
}

constexpr int16_t coord2int(double coord) {
  return static_cast<int16_t>(std::round(coord * 10.0));
}

constexpr double int2angle(int16_t data) { return data / 100.0; }

constexpr double int2coord(int16_t data) { return data / 10.0; }

// Useful for debugging
std::string format_msg(std::string const& data);

}  // namespace mycobot
