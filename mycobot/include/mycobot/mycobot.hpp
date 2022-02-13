#pragma once

#include "mycobot/process_received.hpp"
#include "mycobot/protocol_code.hpp"
#include "mycobot/serialize.hpp"

namespace mycobot {

class MyCobot {
 public:
  MyCobot();

  virtual ~MyCobot();
};

}  // namespace mycobot
