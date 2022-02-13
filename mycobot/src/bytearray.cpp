#include "mycobot/bytearray.hpp"

#include <string>

namespace mycobot {

bytearray to_bytearray(std::string const& str) {
  return bytearray(str.begin(), str.end());
}

}  // namespace mycobot
