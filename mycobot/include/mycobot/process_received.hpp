#pragma once

#include <cmath>
#include <fp/all.hpp>
#include <string>
#include <vector>

#include "mycobot/bytearray.hpp"
#include "mycobot/protocol_code.hpp"

namespace mycobot {

bytearray process_ssid_pwd_response(bytearray const& data);

fp::Result<std::pair<size_t, size_t>> process_header(bytearray const& data,
                                                     ProtocolCode genre);

std::vector<int16_t> process_command(bytearray const& data, ProtocolCode genre);

fp::Result<std::vector<int16_t>> process_received(bytearray const& data,
                                                  ProtocolCode genre);

}  // namespace mycobot
