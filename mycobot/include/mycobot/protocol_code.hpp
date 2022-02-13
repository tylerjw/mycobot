#pragma once

#include <cstdint>

namespace mycobot {

constexpr int8_t safeConvert(uint8_t x) { return x < 128 ? x : x - 256; }

enum class ProtocolCode : int8_t {
  // BASIC
  HEADER = safeConvert(0xFE),
  FOOTER = safeConvert(0xFA),

  // System status
  VERSION = safeConvert(0x00),

  // Overall status
  POWER_ON = safeConvert(0x10),
  POWER_OFF = safeConvert(0x11),
  IS_POWER_ON = safeConvert(0x12),
  RELEASE_ALL_SERVOS = safeConvert(0x13),
  IS_CONTROLLER_CONNECTED = safeConvert(0x14),
  READ_NEXT_ERROR = safeConvert(0x15),
  SET_FREE_MODE = safeConvert(0x1A),
  IS_FREE_MODE = safeConvert(0x1B),

  // MDI MODE AND OPERATION
  GET_ANGLES = safeConvert(0x20),
  SEND_ANGLE = safeConvert(0x21),
  SEND_ANGLES = safeConvert(0x22),
  GET_COORDS = safeConvert(0x23),
  SEND_COORD = safeConvert(0x24),
  SEND_COORDS = safeConvert(0x25),
  PAUSE = safeConvert(0x26),
  IS_PAUSED = safeConvert(0x27),
  RESUME = safeConvert(0x28),
  STOP = safeConvert(0x29),
  IS_IN_POSITION = safeConvert(0x2A),
  IS_MOVING = safeConvert(0x2B),

  // JOG MODE AND OPERATION
  JOG_ANGLE = safeConvert(0x30),
  JOG_COORD = safeConvert(0x32),
  JOG_STOP = safeConvert(0x34),
  SET_ENCODER = safeConvert(0x3A),
  GET_ENCODER = safeConvert(0x3B),
  SET_ENCODERS = safeConvert(0x3C),
  GET_ENCODERS = safeConvert(0x3D),

  // RUNNING STATUS AND SETTINGS
  GET_SPEED = safeConvert(0x40),
  SET_SPEED = safeConvert(0x41),
  GET_FEED_OVERRIDE = safeConvert(0x42),
  GET_ACCELERATION = safeConvert(0x44),
  GET_JOINT_MIN_ANGLE = safeConvert(0x4A),
  GET_JOINT_MAX_ANGLE = safeConvert(0x4B),

  // SERVO CONTROL
  IS_SERVO_ENABLE = safeConvert(0x50),
  IS_ALL_SERVO_ENABLE = safeConvert(0x51),
  SET_SERVO_DATA = safeConvert(0x52),
  GET_SERVO_DATA = safeConvert(0x53),
  SET_SERVO_CALIBRATION = safeConvert(0x54),
  RELEASE_SERVO = safeConvert(0x56),
  FOCUS_SERVO = safeConvert(0x57),

  // ATOM IO
  SET_PIN_MODE = safeConvert(0x60),
  SET_DIGITAL_OUTPUT = safeConvert(0x61),
  GET_DIGITAL_INPUT = safeConvert(0x62),
  SET_PWM_MODE = safeConvert(0x63),
  SET_PWM_OUTPUT = safeConvert(0x64),
  GET_GRIPPER_VALUE = safeConvert(0x65),
  SET_GRIPPER_STATE = safeConvert(0x66),
  SET_GRIPPER_VALUE = safeConvert(0x67),
  SET_GRIPPER_INI = safeConvert(0x68),
  IS_GRIPPER_MOVING = safeConvert(0x69),
  SET_COLOR = safeConvert(0x6A),

  // Basic
  SET_BASIC_OUTPUT = safeConvert(0xA0),
  GET_BASIC_INPUT = safeConvert(0xA1),

  // Linux GPIO, mode: GPIO.BCM
  SET_GPIO_MODE = safeConvert(0xAA),
  SET_GPIO_UP = safeConvert(0xAB),
  SET_GPIO_OUTPUT = safeConvert(0xAC),
  GET_GPIO_IN = safeConvert(0xAD),

  // set WIFI
  SET_SSID_PWD = safeConvert(0xB0),
  GET_SSID_PWD = safeConvert(0xB1),
  SET_SERVER_PORT = safeConvert(0xB2),

  // Get the measured distance
  GET_TOF_DISTANCE = safeConvert(0xC0),
};

constexpr bool operator==(int8_t val, ProtocolCode code) {
  return val == static_cast<int8_t>(code);
}
constexpr bool operator==(ProtocolCode code, int8_t val) {
  return val == static_cast<int8_t>(code);
}
constexpr ProtocolCode to_code(int8_t val) {
  return static_cast<ProtocolCode>(val);
}
constexpr char to_int8(ProtocolCode code) { return static_cast<int8_t>(code); }

}  // namespace mycobot
