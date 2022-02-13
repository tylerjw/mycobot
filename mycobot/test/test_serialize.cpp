#include <catch2/catch.hpp>
#include <limits>
#include <string>

#include "mycobot/serialize.hpp"

namespace mycobot {
SCENARIO("serialize functions serialize data for serial communication",
         "[serialize]") {
  GIVEN("An int8_t value") {
    auto const value =
        GENERATE(take(10, random(std::numeric_limits<int8_t>::min(),
                                 std::numeric_limits<int8_t>::max())));

    WHEN("We encode the value") {
      auto const data = encode(value);

      THEN("The data should be std::string{value}") {
        REQUIRE(data == std::string{value});
      }
    }

    WHEN("We encode the value and convert it to a std::string") {
      auto const encoded_data = encode(value);

      THEN("The encoded data should have a size of 1") {
        REQUIRE(encoded_data.size() == 1);
      }
    }

    WHEN("We encode and then decode the value") {
      auto const decoded_value = decode_int8(encode(value));

      THEN("The decoded value the same as the value") {
        REQUIRE(decoded_value == value);
      }
    }
  }

  GIVEN("An int16_t value") {
    auto const value =
        GENERATE(take(1000, random(std::numeric_limits<int16_t>::min(),
                                   std::numeric_limits<int16_t>::max())));

    WHEN("We encode the value") {
      auto const data = encode(value);

      THEN(
          "The data should be std::string{(value >> 8) & 0xFF, value & 0xFF}") {
        REQUIRE(data == std::string{static_cast<char>((value >> 8) & 0xFF),
                                    static_cast<char>(value & 0xFF)});
      }
    }

    WHEN("We encode the value and convert it to a std::string") {
      auto const encoded_data = encode(value);

      THEN("The encoded data should have a size of 2") {
        REQUIRE(encoded_data.size() == 2);
      }
    }

    WHEN("We encode and then decode the value") {
      auto const decoded_value = decode_int16(encode(value));

      THEN("The decoded value the same as the value") {
        REQUIRE(decoded_value == value);
      }
    }

    WHEN("We convert to angle") {
      auto const angle = int2angle(value);

      THEN("We expect it to be near the value / 100.0") {
        REQUIRE(angle == Approx(value / 100.0));
      }
    }

    WHEN("We convert to angle then convert back to value") {
      auto const decoded_value = angle2int(int2angle(value));

      THEN("We expect it to be near the same value") {
        REQUIRE(decoded_value == Approx(value).margin(1));
      }
    }

    WHEN("We convert to coord") {
      auto const coord = int2coord(value);

      THEN("We expect it to be near the value / 10.0") {
        REQUIRE(coord == Approx(value / 10.0));
      }
    }

    WHEN("We convert to coord then convert back to value") {
      auto const decoded_value = coord2int(int2coord(value));

      THEN("We expect it to be near the same value") {
        REQUIRE(decoded_value == Approx(value).margin(1));
      }
    }
  }

  GIVEN("A double angle in the range -320 to 320") {
    auto const angle = GENERATE(take(100, random(-320.0, 320.0)));

    WHEN("We convert it to a int value") {
      auto const value = angle2int(angle);

      THEN("We expect it to be near 100 * angle") {
        REQUIRE(value == Approx(angle * 100.0).margin(1));
      }
    }
  }

  GIVEN("A double coord in the range -3200 to 3200") {
    auto const coord = GENERATE(take(1000, random(-3200.0, 3200.0)));

    WHEN("We convert it to a int value") {
      auto const value = coord2int(coord);

      THEN("We expect it to be near 10 * coord") {
        REQUIRE(value == Approx(coord * 10.0).margin(1));
      }
    }
  }

  GIVEN("A int8_t, int16_t, and a char") {
    int8_t const a =
        GENERATE(take(10, random(std::numeric_limits<int8_t>::min(),
                                 std::numeric_limits<int8_t>::max())));
    int16_t const b =
        GENERATE(take(10, random(std::numeric_limits<int16_t>::min(),
                                 std::numeric_limits<int16_t>::max())));
    char const c = GENERATE(take(10, random(std::numeric_limits<char>::min(),
                                            std::numeric_limits<char>::max())));

    WHEN("We use the variadic encode to encode them into a std::string") {
      std::string const data = encode(a, b, c);

      THEN("We expect the size to be 4") { REQUIRE(data.size() == 4); }
    }
  }
}

}  // namespace mycobot
