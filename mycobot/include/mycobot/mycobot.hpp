#pragma once

#include <serial/serial.h>

#include <array>
#include <fp/all.hpp>
#include <memory>
#include <string>

#include "mycobot/command.hpp"
#include "mycobot/process_received.hpp"
#include "mycobot/protocol_code.hpp"
#include "mycobot/serialize.hpp"

namespace mycobot {

class MyCobot {
  std::unique_ptr<serial::Serial> serial_port_;

 public:
  /**
   * @brief      Construct MyCobot with serial port settings.
   *
   * @param      port      The port
   * @param[in]  baudrate  The baudrate
   */
  MyCobot(std::string const& port = "/dev/ttyAMA0", uint32_t baudrate = 115200);

  /**
   * @brief      Constructs a new instance with an already configured Serial.
   *
   * @param      serial  The serial connection
   */
  MyCobot(std::unique_ptr<serial::Serial> serial_port);

  /**
   * @brief      Send a Command to the robot
   *
   * @param      command  The command
   *
   * @return     Error if encountered, result if there is one, otherwise empty
   * repose
   */
  fp::Result<response_t> send(Command const& command);

  /**
   * @brief      Gets the radians angles of the joints.
   *
   * @return     The radians.
   */
  fp::Result<std::array<double, 6>> get_radians();

  /**
   * @brief      Send a joint command with radian values
   *
   * @param[in]  radians  The radians
   * @param[in]  speed    The speed (0 ~ 100)
   *
   * @return     Error if command failed, otherwise empty response
   */
  fp::Result<response_t> send_radians(std::array<double, 6> const& radians,
                                      int8_t speed);
};

/**
 * @brief      Attempt to create a Serial coonection to robot by auto-detecting
 * the port
 *
 * @return     Serial coonection to robot or Error
 */
fp::Result<std::unique_ptr<serial::Serial>> make_serial_connection_to_robot();

}  // namespace mycobot
