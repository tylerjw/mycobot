#include "mycobot/serialize.hpp"

#include <cppystruct/cppystruct.h>

#include <fp/all.hpp>
#include <range/v3/all.hpp>
#include <string>
#include <utility>

namespace views = ::ranges::views;

namespace mycobot {

int8_t decode_int8(std::string const& data) {
  return std::get<0>(pystruct::unpack(
      PY_STRING("b"),
      std::string_view(reinterpret_cast<char const*>(data.data()),
                       data.size())));
}

int16_t decode_int16(std::string const& data) {
  return std::get<0>(pystruct::unpack(
      PY_STRING(">h"),
      std::string_view(reinterpret_cast<char const*>(data.data()),
                       data.size())));
}

std::string encode(char data) {
  return std::string(1, pystruct::pack(PY_STRING("b"), data)[0]);
}

std::string encode(int8_t data) {
  return std::string(1, pystruct::pack(PY_STRING("b"), data)[0]);
}

std::string encode(int16_t data) {
  auto const packed_data = pystruct::pack(PY_STRING(">h"), data);
  return std::string{packed_data[0], packed_data[1]};
}

std::string format_msg(std::string const& data) {
  std::string ret;
  for (char const& c : data) {
    ret += fmt::format("{:#x} ", static_cast<uint8_t>(c));
  }
  return ret;
}

}  // namespace mycobot
