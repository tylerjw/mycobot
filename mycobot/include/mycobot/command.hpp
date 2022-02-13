#pragma once

#include <array>
#include <string>

#include "mycobot/protocol_code.hpp"

namespace mycobot {

struct Command {
  ProtocolCode genre;
  std::string data = "";
  bool has_reply = false;
};

Command version();
Command power_on();
Command power_off();
Command is_power_on();
Command release_all_servos();
Command is_controller_connected();
Command get_angles();
Command send_angle(int8_t id, double degrees, int8_t speed);
Command send_angles(std::array<double, 6> const& degrees, int8_t speed);
Command get_coords();
Command send_coord(int8_t id, double coord, int8_t speed);
Command send_coords(std::array<double, 6> const& coords, int8_t speed,
                    int8_t mode);
Command is_in_position(bool coords, std::array<double, 6> const& data);
Command is_moving();
Command jog_angle(int8_t joint_id, int8_t direction, int8_t speed);
Command jog_coord(char coord_id, int8_t direction, int8_t speed);
Command jog_stop();
Command pause();
Command is_paused();
Command resume();
Command stop();
Command set_encoder(int8_t joint_id, int16_t encoder);
Command get_encoder(int8_t joint_id);
Command set_encoders(std::array<int16_t, 6> encoders, int8_t speed);
Command get_encoders();
Command get_speed();
Command set_speed(int8_t speed);
Command get_joint_min_angle(int8_t joint_id);
Command get_joint_max_angle(int8_t joint_id);
Command is_servo_enable(int8_t joint_id);
Command is_all_servo_enable();
Command set_servo_data(int8_t joint_id, int8_t data_id, int8_t value);
Command get_servo_data();
Command set_servo_calibration(int8_t joint_id);
Command release_servo(int8_t joint_id);
Command focus_servo(int8_t joint_id);
Command set_color(uint8_t r, uint8_t g, uint8_t b);
Command set_pin_mode(int8_t pin_no, int8_t pin_mode);
Command set_digital_output(int8_t pin_no, int8_t pin_signal);
Command get_digital_input(int8_t pin_no);
Command set_pwm_output(int8_t channel, int16_t frequency, int8_t pin_val);
Command get_gripper_value();
Command set_gripper_state(int8_t flag, int8_t speed);
Command set_gripper_value(int8_t value, int8_t speed);
Command set_gripper_ini();
Command is_gripper_moving();
Command set_basic_output(int8_t pin_no, int8_t pin_signal);
Command get_basic_input(int8_t pin_no);
Command set_ssid_pwd(std::string const& ssid, std::string const& password);
Command get_ssid_pwd();
Command set_server_port(int8_t port);
Command get_tof_distance();

}  // namespace mycobot
