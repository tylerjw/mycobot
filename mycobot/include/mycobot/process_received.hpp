#pragma once

#include <cmath>
#include <fp/all.hpp>
#include <string>
#include <vector>

#include "mycobot/protocol_code.hpp"

namespace mycobot {

using response_t = std::vector<int16_t>;

std::string process_ssid_pwd_response(std::string const& data);

fp::Result<std::pair<size_t, size_t>> process_header(std::string const& data,
                                                     ProtocolCode genre);

response_t process_command(std::string const& data, ProtocolCode genre);

fp::Result<response_t> process_received(std::string const& data,
                                        ProtocolCode genre);

}  // namespace mycobot
