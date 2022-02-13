#include <mycobot/mycobot.hpp>
#include <mycobot/command.hpp>
#include <fp/all.hpp>

int main()
{
  auto mycobot = mycobot::make_mycobot();

  if (!mycobot) {
    fmt::print("{}\n", mycobot.error());
    return static_cast<int>(mycobot.error().code);
  }

  // set color
  {
    auto const result = mycobot->send(mycobot::set_color(255, 0, 0));
    if(!result) {
      fmt::print("{}\n", result.error());
      return static_cast<int>(result.error().code);
    }
  }

  // get version
  {
    auto const result = mycobot->send(mycobot::version());
    if(!result) {
      fmt::print("{}\n", result.error());
      return static_cast<int>(result.error().code);
    }
    fmt::print("version: {}\n", result.value());
  }

  // read joint angles
  auto const angles = mycobot->send(mycobot::get_angles());
  if(!angles) {
    fmt::print("{}\n", angles.error());
    return static_cast<int>(angles.error().code);
  }
  fmt::print("get_angles: {}\n", angles.value());

  auto const new_angles = std::array<double, 6>{
    angles->at(0) + 2.0,
    angles->at(1) - 2.0,
    angles->at(2) + 2.0,
    angles->at(3) - 2.0,
    angles->at(4) + 2.0,
    angles->at(5) - 2.0,
  };

  auto const result = mycobot->send(mycobot::send_angles(new_angles, 20));
  if(!result) {
    fmt::print("{}\n", result.error());
    return static_cast<int>(result.error().code);
  }

  // release all servos
  {
    auto const result = mycobot->send(mycobot::release_all_servos());
    if(!result) {
      fmt::print("{}\n", result.error());
      return static_cast<int>(result.error().code);
    }
  }

  return 0;
}
