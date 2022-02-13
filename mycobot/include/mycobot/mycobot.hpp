#pragma once

#include <serial/serial.h>

#include <fp/all.hpp>
#include <memory>

#include "mycobot/command.hpp"
#include "mycobot/process_received.hpp"
#include "mycobot/protocol_code.hpp"
#include "mycobot/serialize.hpp"

namespace mycobot {

class MyCobot {
  std::unique_ptr<serial::Serial> serial_port_;

 public:
  /**
   * @brief      Construct MyCobot with serial port settings
   *
   * @param      port      The port
   * @param[in]  baudrate  The baudrate
   */
  MyCobot(std::string const& port = "/dev/ttyAMA0", uint32_t baudrate = 115200)
      : serial_port_{std::make_unique<serial::Serial>(
            port, baudrate, serial::Timeout::simpleTimeout(1000))} {}

  /**
   * @brief      Constructs a new instance with an already configured Serial
   *
   * @param      serial  The serial connection
   */
  MyCobot(std::unique_ptr<serial::Serial> serial_port)
      : serial_port_{std::move(serial_port)} {}

  /**
   * @brief      Send a Command to the robot
   *
   * @param      command  The command
   *
   * @return     Error if encountered, result if there is one, otherwise empty
   * repose
   */
  fp::Result<response_t> send(Command const& command);
};

/**
 * @brief      Attempt to create a MyCobot by auto-detecting the port
 *
 * @return     MyCobot or Error
 */
fp::Result<MyCobot> make_mycobot();

}  // namespace mycobot
