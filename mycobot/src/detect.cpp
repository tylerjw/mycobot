#include "mycobot/detect.hpp"

#include <serial/serial.h>

#include <vector>
#include <string>
#include <optional>

namespace mycobot {

std::vector<std::string> get_ports()
{
  return []() {
    std::vector<std::string> ret;
    for (auto const& port_info : serial::list_ports()) ret.push_back(port_info.port);
    return ret;
  }();
}

std::optional<std::string> get_port_of_robot()
{
  auto const ports = []() {
    std::vector<std::string> ret;
    for (auto const& port_info : serial::list_ports()) {
      if (port_info.hardware_id == "0xEA60") {
        ret.push_back(port_info.port);
      }
    }
    return ret;
  }();

  if (ports.size() > 0) {
    return ports.at(0);
  }
  return std::nullopt;
}

}  // namespace mycobot
