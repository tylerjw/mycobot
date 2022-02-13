#include "mycobot/serialize.hpp"

#include <fp/all.hpp>
#include <range/v3/all.hpp>
#include <string>
#include <utility>

#include "mycobot/bytearray.hpp"

namespace views = ::ranges::views;

namespace mycobot {

int8_t decode_int8(bytearray const& data) {
  return std::get<0>(pystruct::unpack(
      PY_STRING("b"),
      std::string_view(reinterpret_cast<char const*>(data.data()),
                       data.size())));
}

int16_t decode_int16(bytearray const& data) {
  return std::get<0>(pystruct::unpack(
      PY_STRING(">h"),
      std::string_view(reinterpret_cast<char const*>(data.data()),
                       data.size())));
}

bytearray encode(char data) {
  return to_bytearray(pystruct::pack(PY_STRING("b"), data));
}

bytearray encode(int8_t data) {
  return to_bytearray(pystruct::pack(PY_STRING("b"), data));
}

bytearray encode(int16_t data) {
  return to_bytearray(pystruct::pack(PY_STRING(">h"), data));
}

}  // namespace mycobot
