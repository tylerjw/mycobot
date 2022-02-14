#include "mycobot/mycobot.hpp"

#include <serial/serial.h>

#include <cmath>
#include <fp/all.hpp>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "mycobot/detect.hpp"

namespace mycobot {
namespace {
auto constexpr kBaudrate = 115200;
auto constexpr kTimeout = 100;  // Milliseconds
}  // namespace

MyCobot::MyCobot(std::string const& port, uint32_t baudrate)
    : serial_port_{std::make_unique<serial::Serial>(
          port, baudrate, serial::Timeout::simpleTimeout(kTimeout))} {}

MyCobot::MyCobot(std::unique_ptr<serial::Serial> serial_port)
    : serial_port_{std::move(serial_port)} {}

fp::Result<response_t> MyCobot::send(Command const& command) {
  // Send the data over the serial port
  try {
    serial_port_->write(command.data);
    serial_port_->flushOutput();
  } catch (serial::PortNotOpenedException const& ex) {
    return tl::make_unexpected(fp::FailedPrecondition(
        fmt::format("Caught serial::PortNotOpenedException: {}", ex.what())));
  } catch (serial::SerialException const& ex) {
    return tl::make_unexpected(fp::Cancelled(
        fmt::format("Caught serial::SerialException: {}", ex.what())));
  } catch (serial::IOException const& ex) {
    return tl::make_unexpected(
        fp::Internal(fmt::format("Caught serial::IOException: {}", ex.what())));
  }

  if (command.has_reply) {
    // Wait until we there is data to read
    auto const readable = serial_port_->waitReadable();
    if (!readable) {
      return tl::make_unexpected(fp::Timeout("Timeout waiting for reply"));
    }

    // Read the reply
    std::vector<std::string> received_data;
    try {
      received_data =
          serial_port_->readlines(65536, encode(to_int8(ProtocolCode::FOOTER)));
    } catch (serial::PortNotOpenedException const& ex) {
      return tl::make_unexpected(fp::FailedPrecondition(
          fmt::format("Caught serial::PortNotOpenedException: {}", ex.what())));
    } catch (serial::SerialException const& ex) {
      return tl::make_unexpected(fp::Cancelled(
          fmt::format("Caught serial::SerialException: {}", ex.what())));
    }

    fp::Error last_error;
    for (auto const& line : received_data) {
      auto const response = process_received(line, command.genre);

      if (!response) {
        last_error = response.error();
      } else {
        return response.value();
      }
    }
    serial_port_->flushInput();
    return tl::make_unexpected(last_error);
  }

  // Return an empty response_t if the command does not have a reply
  return {};
}

fp::Result<std::array<double, 6>> MyCobot::get_radians() {
  auto const raw_angles = send(get_angles());
  if (!raw_angles) {
    return tl::make_unexpected(raw_angles.error());
  }

  return std::array<double, 6>{
      int2angle(raw_angles->at(0)) * M_PI / 180.0,
      int2angle(raw_angles->at(1)) * M_PI / 180.0,
      int2angle(raw_angles->at(2)) * M_PI / 180.0,
      int2angle(raw_angles->at(3)) * M_PI / 180.0,
      int2angle(raw_angles->at(4)) * M_PI / 180.0,
      int2angle(raw_angles->at(5)) * M_PI / 180.0,
  };
}

fp::Result<response_t> MyCobot::send_radians(
    std::array<double, 6> const& radians, int8_t speed) {
  return send(send_angles(
      std::array<double, 6>{
          radians.at(0) * 180.0 / M_PI,
          radians.at(1) * 180.0 / M_PI,
          radians.at(2) * 180.0 / M_PI,
          radians.at(3) * 180.0 / M_PI,
          radians.at(4) * 180.0 / M_PI,
          radians.at(5) * 180.0 / M_PI,
      },
      speed));
}

fp::Result<std::unique_ptr<serial::Serial>> make_serial_connection_to_robot() {
  auto const maybe_port = get_port_of_robot();
  if (!maybe_port) {
    return tl::make_unexpected(fp::NotFound("Port of MyCobot not found"));
  }

  auto serial_port = std::make_unique<serial::Serial>();
  serial_port->setPort(maybe_port.value());
  serial_port->setBaudrate(kBaudrate);
  auto timeout = serial::Timeout::simpleTimeout(kTimeout);
  serial_port->setTimeout(timeout);
  serial_port->open();

  if (!serial_port->isOpen()) {
    return tl::make_unexpected(fp::Unavailable(
        fmt::format("Attempt to connect to MyCobot failed at port: {}",
                    maybe_port.value())));
  }

  return serial_port;
}

}  // namespace mycobot
