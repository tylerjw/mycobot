#include <chrono>
#include <fp/all.hpp>
#include <mycobot/command.hpp>
#include <mycobot/mycobot.hpp>
#include <thread>

int main() {
  auto mycobot = mycobot::make_mycobot();

  if (!mycobot) {
    fmt::print("{}\n", mycobot.error());
    return static_cast<int>(mycobot.error().code);
  }

  // power on
  {
    auto const result = mycobot->send(mycobot::power_on());
    if (!result) {
      fmt::print("{}\n", result.error());
      return static_cast<int>(result.error().code);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // move to origin
  {
    auto const result = mycobot->send(
        mycobot::send_angles(std::array<double, 6>{0, 0, 0, 0, 0, 0}, 50));
    if (!result) {
      fmt::print("{}\n", result.error());
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  // read joint angles
  auto const angles = mycobot->send(mycobot::get_angles());
  if (!angles) {
    fmt::print("{}\n", angles.error());
    // return static_cast<int>(angles.error().code);
  } else {
    fmt::print("get_angles: {}\n", angles.value());
  }

  // move a bit
  {
    auto const result = mycobot->send(mycobot::send_angles(
        std::array<double, 6>{
            mycobot::int2angle(angles->at(0)) + 1.0,
            mycobot::int2angle(angles->at(1)) - 1.0,
            mycobot::int2angle(angles->at(2)) + 1.0,
            mycobot::int2angle(angles->at(3)) - 1.0,
            mycobot::int2angle(angles->at(4)) + 1.0,
            mycobot::int2angle(angles->at(5)) - 1.0,
        },
        100));
    if (!result) {
      fmt::print("{}\n", result.error());
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  // // release all servos
  {
    auto const result = mycobot->send(mycobot::release_all_servos());
    if (!result) {
      fmt::print("{}\n", result.error());
    }
  }

  // power off
  {
    auto const result = mycobot->send(mycobot::power_off());
    if (!result) {
      fmt::print("{}\n", result.error());
      return static_cast<int>(result.error().code);
    }
  }

  return 0;
}
