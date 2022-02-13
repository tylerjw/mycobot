#include <fp/all.hpp>
#include <optional>
#include <serial/serial.h>
#include <memory>
#include <string>

#include "mycobot/mycobot.hpp"
#include "mycobot/detect.hpp"

namespace mycobot {

fp::Result<MyCobot> make_mycobot()
{
  auto const maybe_port = get_port_of_robot();
  if(!maybe_port) {
    return tl::make_unexpected(fp::NotFound("Port of MyCobot not found"));
  }

  auto serial_port = std::make_unique<serial::Serial>();
  serial_port->setPort(maybe_port.value());
  // serial_port->setPort("/dev/ttyDUMMY");
  serial_port->setBaudrate(115200);
  auto timeout = serial::Timeout::simpleTimeout(10000);
  serial_port->setTimeout(timeout);
  serial_port->open();

  if (!serial_port->isOpen()) {
    return tl::make_unexpected(fp::Unavailable(
      fmt::format("Attempt to connect to MyCobot failed at port: {}", maybe_port.value())));
  }

  return MyCobot(std::move(serial_port));
}

fp::Result<response_t> MyCobot::send(Command const& command)
{
  fmt::print("send: {}\n", format_msg(command.data));

  // Send the data over the serial port
  try {
    serial_port_->write(command.data);
    serial_port_->flushOutput();
  } catch(serial::PortNotOpenedException const& ex) {
    return tl::make_unexpected(fp::FailedPrecondition(
      fmt::format("Caught serial::PortNotOpenedException: {}", ex.what())));
  } catch(serial::SerialException const& ex) {
    return tl::make_unexpected(fp::Cancelled(
      fmt::format("Caught serial::SerialException: {}", ex.what())));
  } catch(serial::IOException const& ex) {
    return tl::make_unexpected(fp::Internal(
      fmt::format("Caught serial::IOException: {}", ex.what())));
  }

  if(command.has_reply) {
    // Wait until we there is data to read
    // auto const readable = serial_port_->waitReadable();
    // if(!readable) {
    //   return tl::make_unexpected(fp::Timeout("Timeout waiting for reply"));
    // }

    // Read the reply
    std::string received_data;
    try {
      received_data = serial_port_->readline(65536,
        encode(to_int8(ProtocolCode::FOOTER)));
    } catch(serial::PortNotOpenedException const& ex) {
      return tl::make_unexpected(fp::FailedPrecondition(
        fmt::format("Caught serial::PortNotOpenedException: {}", ex.what())));
    } catch(serial::SerialException const& ex) {
      return tl::make_unexpected(fp::Cancelled(
        fmt::format("Caught serial::SerialException: {}", ex.what())));
    }

    fmt::print("received: {}\n", format_msg(received_data));

    auto const response = process_received(received_data, command.genre);

    if(!response) {
      return tl::make_unexpected(response.error());
    }

    return response.value();
  }

  // Return an empty response_t if the command does not have a reply
  return {};
}

}  // namespace mycobot
