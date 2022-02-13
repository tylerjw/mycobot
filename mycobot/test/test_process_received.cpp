#include <string>
#include <array>
#include <catch2/catch.hpp>
#include <limits>
#include <range/v3/all.hpp>

#include "mycobot/process_received.hpp"
#include "mycobot/serialize.hpp"

namespace views = ::ranges::views;

namespace mycobot {
SCENARIO("process received functions process received data from robot",
         "[process_received]") {
  GIVEN("A valid received to is power on") {
    auto const genre = ProtocolCode::IS_POWER_ON;
    auto const received =
        std::string{to_int8(ProtocolCode::HEADER), to_int8(ProtocolCode::HEADER),
                  1, to_int8(genre), 1};

    WHEN("We process the header") {
      auto const result = process_header(received, genre);
      REQUIRE(result.has_value());

      THEN("The index is 4 and the data_len is 1") {
        auto const& [start_index, data_len] = result.value();
        REQUIRE(start_index == 4);
        REQUIRE(data_len == 1);
      }
    }

    WHEN("We process the received") {
      auto const result = process_received(received, genre);
      REQUIRE(result.has_value());

      THEN("The result is of size 1 and contains the value 1") {
        REQUIRE(result->size() == 1);
        REQUIRE(result->at(0) == 1);
      }
    }
  }

  GIVEN("A valid received to is power on that starts with some garbage") {
    auto const genre = ProtocolCode::IS_POWER_ON;
    auto const received = std::string{0,
                                    2,
                                    3,
                                    to_int8(ProtocolCode::HEADER),
                                    to_int8(ProtocolCode::HEADER),
                                    1,
                                    to_int8(genre),
                                    1};

    WHEN("We process the header") {
      auto const result = process_header(received, genre);
      REQUIRE(result.has_value());

      THEN("The index is 7 and the data_len is 1") {
        auto const& [start_index, data_len] = result.value();
        REQUIRE(start_index == 7);
        REQUIRE(data_len == 1);
      }
    }

    WHEN("We process the received") {
      auto const result = process_received(received, genre);
      REQUIRE(result.has_value());

      THEN("The result is of size 1 and contains the value 1") {
        REQUIRE(result->size() == 1);
        REQUIRE(result->at(0) == 1);
      }
    }
  }

  GIVEN("A valid received to get encoders") {
    auto const genre = ProtocolCode::GET_ENCODERS;
    std::array<int16_t, 6> encoder_values = {1, 2, 3, 4, 5, 6};
    auto const received = [&]() {
      auto ret = std::string{to_int8(ProtocolCode::HEADER),
                           to_int8(ProtocolCode::HEADER), 12, to_int8(genre)};
      for (auto const& value : encoder_values) {
        ret.append(encode(value));
      }
      return ret;
    }();

    WHEN("We process the header") {
      auto const result = process_header(received, genre);
      REQUIRE(result.has_value());

      THEN("The index is 4 and the data_len is 12") {
        auto const& [start_index, data_len] = result.value();
        REQUIRE(start_index == 4);
        REQUIRE(data_len == 12);
      }
    }

    WHEN("We process the received") {
      auto const result = process_received(received, genre);
      REQUIRE(result.has_value());

      THEN("The result is of size 6 and contains the encoder values") {
        REQUIRE(result->size() == 6);
        REQUIRE(result->at(0) == encoder_values.at(0));
        REQUIRE(result->at(1) == encoder_values.at(1));
        REQUIRE(result->at(2) == encoder_values.at(2));
        REQUIRE(result->at(3) == encoder_values.at(3));
        REQUIRE(result->at(4) == encoder_values.at(4));
        REQUIRE(result->at(5) == encoder_values.at(5));
      }
    }
  }
}

}  // namespace mycobot
