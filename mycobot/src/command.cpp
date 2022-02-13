#include "mycobot/command.hpp"

#include <array>
#include <range/v3/all.hpp>
#include <string>

#include "mycobot/protocol_code.hpp"
#include "mycobot/serialize.hpp"

namespace views = ::ranges::views;

namespace mycobot {
namespace {
std::string make_data(ProtocolCode genre, std::string const& data = {}) {
  return std::string{to_int8(ProtocolCode::HEADER),
                     to_int8(ProtocolCode::HEADER),
                     static_cast<int8_t>(data.size() + 2), to_int8(genre),
                     to_int8(ProtocolCode::FOOTER)}
      .insert(4, data);
}
}  // namespace

Command version() {
  return Command{ProtocolCode::VERSION, make_data(ProtocolCode::VERSION), true};
}

Command power_on() {
  return Command{ProtocolCode::POWER_ON, make_data(ProtocolCode::POWER_ON),
                 false};
}

Command power_off() {
  return Command{ProtocolCode::POWER_OFF, make_data(ProtocolCode::POWER_OFF),
                 false};
}

Command is_power_on() {
  return Command{ProtocolCode::IS_POWER_ON,
                 make_data(ProtocolCode::IS_POWER_ON), true};
}

Command release_all_servos() {
  return Command{ProtocolCode::RELEASE_ALL_SERVOS,
                 make_data(ProtocolCode::RELEASE_ALL_SERVOS), false};
}

Command is_controller_connected() {
  return Command{ProtocolCode::IS_CONTROLLER_CONNECTED,
                 make_data(ProtocolCode::IS_CONTROLLER_CONNECTED), true};
}

Command get_angles() {
  return Command{ProtocolCode::GET_ANGLES, make_data(ProtocolCode::GET_ANGLES),
                 true};
}

Command send_angle(int8_t id, double degrees, int8_t speed) {
  auto const data = encode(id, angle2int(degrees), speed);
  return Command{ProtocolCode::SEND_ANGLE,
                 make_data(ProtocolCode::SEND_ANGLE, data), false};
}

Command send_angles(std::array<double, 6> const& degrees, int8_t speed) {
  return Command{
      ProtocolCode::SEND_ANGLES,
      make_data(ProtocolCode::SEND_ANGLES,
                encode(angle2int(degrees[0]), angle2int(degrees[1]),
                       angle2int(degrees[2]), angle2int(degrees[3]),
                       angle2int(degrees[4]), angle2int(degrees[5]), speed)),
      false};
}

Command get_coords() {
  return Command{ProtocolCode::GET_COORDS, make_data(ProtocolCode::GET_COORDS),
                 true};
}

Command send_coord(int8_t id, double coord, int8_t speed) {
  auto const data =
      encode(id, id <= 3 ? coord2int(coord) : angle2int(coord), speed);
  return Command{ProtocolCode::SEND_COORD,
                 make_data(ProtocolCode::SEND_COORD, data), false};
}

Command send_coords(std::array<double, 6> const& coords, int8_t speed,
                    int8_t mode) {
  return Command{ProtocolCode::SEND_COORDS,
                 make_data(ProtocolCode::SEND_COORDS,
                           encode(coord2int(coords[0]), coord2int(coords[1]),
                                  coord2int(coords[2]), angle2int(coords[3]),
                                  angle2int(coords[4]), angle2int(coords[5]),
                                  speed, mode)),
                 false};
}

Command is_in_position(bool coords, std::array<double, 6> const& data) {
  return Command{
      ProtocolCode::IS_IN_POSITION,
      make_data(ProtocolCode::IS_IN_POSITION,
                [&]() {
                  auto ret = std::string{};
                  if (coords) {
                    return encode(coord2int(data[0]), coord2int(data[1]),
                                  coord2int(data[2]), angle2int(data[3]),
                                  angle2int(data[4]), angle2int(data[5]));
                  } else {
                    return encode(angle2int(data[0]), angle2int(data[1]),
                                  angle2int(data[2]), angle2int(data[3]),
                                  angle2int(data[4]), angle2int(data[5]));
                  }
                  return ret;
                }()),
      true};
}

Command is_moving() {
  return Command{ProtocolCode::IS_MOVING, make_data(ProtocolCode::IS_MOVING),
                 true};
}

Command jog_angle(int8_t joint_id, int8_t direction, int8_t speed) {
  return Command{
      ProtocolCode::JOG_ANGLE,
      make_data(ProtocolCode::JOG_ANGLE, encode(joint_id, direction, speed)),
      false};
}

Command jog_coord(char coord_id, int8_t direction, int8_t speed) {
  return Command{
      ProtocolCode::JOG_COORD,
      make_data(ProtocolCode::JOG_COORD, encode(coord_id, direction, speed)),
      false};
}

Command jog_stop() {
  return Command{ProtocolCode::JOG_STOP, make_data(ProtocolCode::JOG_STOP),
                 false};
}

Command pause() {
  return Command{ProtocolCode::PAUSE, make_data(ProtocolCode::PAUSE), false};
}

Command is_paused() {
  return Command{ProtocolCode::IS_PAUSED, make_data(ProtocolCode::IS_PAUSED),
                 true};
}

Command resume() {
  return Command{ProtocolCode::RESUME, make_data(ProtocolCode::RESUME), false};
}

Command stop() {
  return Command{ProtocolCode::STOP, make_data(ProtocolCode::STOP), false};
}

Command set_encoder(int8_t joint_id, int16_t encoder) {
  return Command{
      ProtocolCode::SET_ENCODER,
      make_data(ProtocolCode::SET_ENCODER, encode(joint_id, encoder)), false};
}

Command get_encoder(int8_t joint_id) {
  return Command{ProtocolCode::GET_ENCODER,
                 make_data(ProtocolCode::GET_ENCODER, encode(joint_id)), true};
}

Command set_encoders(std::array<int16_t, 6> encoders, int8_t speed) {
  return Command{
      ProtocolCode::SET_ENCODERS,
      make_data(ProtocolCode::SET_ENCODERS,
                encode(encoders[0], encoders[1], encoders[2], encoders[3],
                       encoders[4], encoders[5], encoders[6], speed)),
      false};
}

Command get_encoders() {
  return Command{ProtocolCode::GET_ENCODERS,
                 make_data(ProtocolCode::GET_ENCODERS), true};
}

Command get_speed() {
  return Command{ProtocolCode::GET_SPEED, make_data(ProtocolCode::GET_SPEED),
                 true};
}

Command set_speed(int8_t speed) {
  return Command{ProtocolCode::SET_SPEED,
                 make_data(ProtocolCode::SET_SPEED, encode(speed)), false};
}

Command get_joint_min_angle(int8_t joint_id) {
  return Command{ProtocolCode::GET_JOINT_MIN_ANGLE,
                 make_data(ProtocolCode::GET_JOINT_MIN_ANGLE, encode(joint_id)),
                 true};
}

Command get_joint_max_angle(int8_t joint_id) {
  return Command{ProtocolCode::GET_JOINT_MAX_ANGLE,
                 make_data(ProtocolCode::GET_JOINT_MAX_ANGLE, encode(joint_id)),
                 true};
}

Command is_servo_enable(int8_t joint_id) {
  return Command{ProtocolCode::IS_SERVO_ENABLE,
                 make_data(ProtocolCode::IS_SERVO_ENABLE, encode(joint_id)),
                 true};
}

Command is_all_servo_enable() {
  return Command{ProtocolCode::IS_ALL_SERVO_ENABLE,
                 make_data(ProtocolCode::IS_ALL_SERVO_ENABLE), true};
}

Command set_servo_data(int8_t joint_id, int8_t data_id, int8_t value) {
  return Command{
      ProtocolCode::SET_SERVO_DATA,
      make_data(ProtocolCode::SET_SERVO_DATA, encode(joint_id, data_id, value)),
      false};
}

Command get_servo_data() {
  return Command{ProtocolCode::GET_SERVO_DATA,
                 make_data(ProtocolCode::GET_SERVO_DATA), true};
}

Command set_servo_calibration(int8_t joint_id) {
  return Command{
      ProtocolCode::SET_SERVO_CALIBRATION,
      make_data(ProtocolCode::SET_SERVO_CALIBRATION, encode(joint_id)), false};
}

Command release_servo(int8_t joint_id) {
  return Command{ProtocolCode::RELEASE_SERVO,
                 make_data(ProtocolCode::RELEASE_SERVO, encode(joint_id)),
                 false};
}

Command focus_servo(int8_t joint_id) {
  return Command{ProtocolCode::FOCUS_SERVO,
                 make_data(ProtocolCode::FOCUS_SERVO, encode(joint_id)), false};
}

Command set_color(uint8_t r, uint8_t g, uint8_t b) {
  return Command{
      ProtocolCode::SET_COLOR,
      make_data(ProtocolCode::SET_COLOR,
                encode(safeConvert(r), safeConvert(g), safeConvert(b))),
      false};
}

Command set_pin_mode(int8_t pin_no, int8_t pin_mode) {
  return Command{
      ProtocolCode::SET_PIN_MODE,
      make_data(ProtocolCode::SET_PIN_MODE, encode(pin_no, pin_mode)), false};
}

Command set_digital_output(int8_t pin_no, int8_t pin_signal) {
  return Command{
      ProtocolCode::SET_DIGITAL_OUTPUT,
      make_data(ProtocolCode::SET_DIGITAL_OUTPUT, encode(pin_no, pin_signal)),
      false};
}

Command get_digital_input(int8_t pin_no) {
  return Command{ProtocolCode::GET_DIGITAL_INPUT,
                 make_data(ProtocolCode::GET_DIGITAL_INPUT, encode(pin_no)),
                 true};
}

Command set_pwm_output(int8_t channel, int16_t frequency, int8_t pin_val) {
  return Command{ProtocolCode::SET_PWM_OUTPUT,
                 make_data(ProtocolCode::SET_PWM_OUTPUT,
                           encode(channel, frequency, pin_val)),
                 false};
}

Command get_gripper_value() {
  return Command{ProtocolCode::GET_GRIPPER_VALUE,
                 make_data(ProtocolCode::GET_GRIPPER_VALUE), true};
}

Command set_gripper_state(int8_t flag, int8_t speed) {
  return Command{
      ProtocolCode::SET_GRIPPER_STATE,
      make_data(ProtocolCode::SET_GRIPPER_STATE, encode(flag, speed)), false};
}

Command set_gripper_value(int8_t value, int8_t speed) {
  return Command{
      ProtocolCode::SET_GRIPPER_VALUE,
      make_data(ProtocolCode::SET_GRIPPER_VALUE, encode(value, speed)), false};
}

Command set_gripper_ini() {
  return Command{ProtocolCode::SET_GRIPPER_INI,
                 make_data(ProtocolCode::SET_GRIPPER_INI), false};
}

Command is_gripper_moving() {
  return Command{ProtocolCode::IS_GRIPPER_MOVING,
                 make_data(ProtocolCode::IS_GRIPPER_MOVING), true};
}

Command set_basic_output(int8_t pin_no, int8_t pin_signal) {
  return Command{
      ProtocolCode::SET_BASIC_OUTPUT,
      make_data(ProtocolCode::SET_BASIC_OUTPUT, encode(pin_no, pin_signal)),
      false};
}

Command get_basic_input(int8_t pin_no) {
  return Command{ProtocolCode::GET_BASIC_INPUT,
                 make_data(ProtocolCode::GET_BASIC_INPUT, encode(pin_no)),
                 true};
}

Command set_ssid_pwd(std::string const& ssid, std::string const& password) {
  return Command{
      ProtocolCode::SET_SSID_PWD,
      make_data(ProtocolCode::SET_SSID_PWD, std::string(ssid).append(password)),
      false};
}

Command get_ssid_pwd() {
  return Command{ProtocolCode::GET_SSID_PWD,
                 make_data(ProtocolCode::GET_SSID_PWD), true};
}

Command set_server_port(int8_t port) {
  return Command{ProtocolCode::SET_SERVER_PORT,
                 make_data(ProtocolCode::SET_SERVER_PORT, encode(port)), false};
}

Command get_tof_distance() {
  return Command{ProtocolCode::GET_TOF_DISTANCE,
                 make_data(ProtocolCode::GET_TOF_DISTANCE), true};
}

}  // namespace mycobot
