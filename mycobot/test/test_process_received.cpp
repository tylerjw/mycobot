#include <array>
#include <catch2/catch.hpp>
#include <limits>
#include <range/v3/all.hpp>
#include <string>

#include "mycobot/process_received.hpp"
#include "mycobot/serialize.hpp"

namespace views = ::ranges::views;

namespace mycobot {
SCENARIO("process received functions process received data from robot",
         "[process_received]") {
  GIVEN("A valid received to is power on") {
    auto const genre = ProtocolCode::IS_POWER_ON;
    auto const received =
        std::string{to_int8(ProtocolCode::HEADER),
                    to_int8(ProtocolCode::HEADER), 3, to_int8(genre), 1};

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
                                      3,
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
                             to_int8(ProtocolCode::HEADER), 14, to_int8(genre)};
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

  GIVEN("Some actual data received from the robot") {
    auto const received =
        encode(static_cast<int8_t>(0x0), static_cast<int8_t>(0x0),
               static_cast<int8_t>(0x8), static_cast<int8_t>(0x0),
               static_cast<int8_t>(0x0), static_cast<int8_t>(0xf4),
               static_cast<int8_t>(0x1), static_cast<int8_t>(0x40),
               static_cast<int8_t>(0xff), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0xfe), static_cast<int8_t>(0xc),
               static_cast<int8_t>(0x83), static_cast<int8_t>(0x29),
               static_cast<int8_t>(0x7), static_cast<int8_t>(0x6),
               static_cast<int8_t>(0x0), static_cast<int8_t>(0x0),
               static_cast<int8_t>(0x8), static_cast<int8_t>(0x0),
               static_cast<int8_t>(0x0), static_cast<int8_t>(0x4d),
               static_cast<int8_t>(0x1), static_cast<int8_t>(0xe6),
               static_cast<int8_t>(0xff), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0x1), static_cast<int8_t>(0x4),
               static_cast<int8_t>(0x2), static_cast<int8_t>(0x38),
               static_cast<int8_t>(0x2), static_cast<int8_t>(0xbe),
               static_cast<int8_t>(0xff), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0x1), static_cast<int8_t>(0x4),
               static_cast<int8_t>(0x0), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0x7), static_cast<int8_t>(0xf4),
               static_cast<int8_t>(0xff), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0x2), static_cast<int8_t>(0x4),
               static_cast<int8_t>(0x2), static_cast<int8_t>(0x38),
               static_cast<int8_t>(0x2), static_cast<int8_t>(0xbd),
               static_cast<int8_t>(0xff), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0x2), static_cast<int8_t>(0x4),
               static_cast<int8_t>(0x0), static_cast<int8_t>(0x3),
               static_cast<int8_t>(0x8), static_cast<int8_t>(0xee),
               static_cast<int8_t>(0xff), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0x3), static_cast<int8_t>(0x4),
               static_cast<int8_t>(0x2), static_cast<int8_t>(0x38),
               static_cast<int8_t>(0x2), static_cast<int8_t>(0xbc),
               static_cast<int8_t>(0xff), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0x3), static_cast<int8_t>(0x4),
               static_cast<int8_t>(0x0), static_cast<int8_t>(0xf9),
               static_cast<int8_t>(0x7), static_cast<int8_t>(0xf8),
               static_cast<int8_t>(0xff), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0x4), static_cast<int8_t>(0x4),
               static_cast<int8_t>(0x2), static_cast<int8_t>(0x38),
               static_cast<int8_t>(0x2), static_cast<int8_t>(0xbb),
               static_cast<int8_t>(0xff), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0x4), static_cast<int8_t>(0x4),
               static_cast<int8_t>(0x0), static_cast<int8_t>(0x5),
               static_cast<int8_t>(0x8), static_cast<int8_t>(0xea),
               static_cast<int8_t>(0xff), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0x5), static_cast<int8_t>(0x4),
               static_cast<int8_t>(0x2), static_cast<int8_t>(0x38),
               static_cast<int8_t>(0x2), static_cast<int8_t>(0xba),
               static_cast<int8_t>(0xff), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0x5), static_cast<int8_t>(0x4),
               static_cast<int8_t>(0x0), static_cast<int8_t>(0xfa),
               static_cast<int8_t>(0x7), static_cast<int8_t>(0xf5),
               static_cast<int8_t>(0xff), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0x6), static_cast<int8_t>(0x4),
               static_cast<int8_t>(0x2), static_cast<int8_t>(0x38),
               static_cast<int8_t>(0x2), static_cast<int8_t>(0xb9),
               static_cast<int8_t>(0xff), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0xff), static_cast<int8_t>(0x6),
               static_cast<int8_t>(0x4), static_cast<int8_t>(0x0),
               static_cast<int8_t>(0x6), static_cast<int8_t>(0x8),
               static_cast<int8_t>(0xe7), static_cast<int8_t>(0xfe),
               static_cast<int8_t>(0xfe), static_cast<int8_t>(0xe),
               static_cast<int8_t>(0x20), static_cast<int8_t>(0x0),
               static_cast<int8_t>(0x8), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0xe6), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0xc3), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0xd5), static_cast<int8_t>(0x0),
               static_cast<int8_t>(0x34), static_cast<int8_t>(0xff),
               static_cast<int8_t>(0xcc), static_cast<int8_t>(0xfa));
    auto const genre = static_cast<ProtocolCode>(0x20);

    WHEN("We process the header") {
      auto const result = process_header(received, genre);
      REQUIRE(result.has_value());

      THEN("The index is 8 and the data_len is 12") {
        auto const& [start_index, data_len] = result.value();
        REQUIRE(start_index == 125);
        REQUIRE(data_len == 12);
      }
    }

    WHEN("We process the received") {
      auto const result = process_received(received, genre);
      REQUIRE(result.has_value());

      THEN("The result is of size 6 and contains the encoder values") {
        REQUIRE(result->size() == 6);
        REQUIRE(int2angle(result->at(0)) == Approx(0.08));
        REQUIRE(int2angle(result->at(1)) == Approx(-0.26));
        REQUIRE(int2angle(result->at(2)) == Approx(-0.61));
        REQUIRE(int2angle(result->at(3)) == Approx(-0.43));
        REQUIRE(int2angle(result->at(4)) == Approx(0.52));
        REQUIRE(int2angle(result->at(5)) == Approx(-0.52));
      }
    }
  }
}

}  // namespace mycobot
