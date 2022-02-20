#include "mycobot/serialize.hpp"

#include <fp/all.hpp>
#include <string>
#include <utility>

namespace mycobot {

int8_t decode_int8(std::string const& data) {
  return static_cast<int8_t>(data.at(0));
}

int16_t decode_int16(std::string const& data) {
  auto const msb = static_cast<int16_t>(static_cast<uint16_t>(data.at(0)) << 8);
  auto const lsb = static_cast<int16_t>(data.at(1) & 0xff);
  return msb + lsb;
}

std::string encode(char data) { return std::string(1, data); }

std::string encode(int8_t data) { return std::string(1, data); }

std::string encode(int16_t data) {
  return std::string{static_cast<char>((data >> 8) & 0xFF),
                     static_cast<char>(data & 0xFF)};
}

std::string format_msg(std::string const& data) {
  std::string ret;
  for (char const& c : data) {
    ret += fmt::format("{:#x} ", static_cast<uint8_t>(c));
  }
  return ret;
}

}  // namespace mycobot
