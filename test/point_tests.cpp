#include <catch2/catch.hpp>

#include <jive/range.h>
#include <jive/testing/cast_limits.h>
#include <jive/testing/generator_limits.h>

#include "tau/point.h"


TEST_CASE("Point converts to int", "[point]")
{
    auto values = GENERATE(
        take(
            10,
            chunk(
                2,
                random(-100, 100))));

    tau::Point<double> point(values.at(0), values.at(1));

    int expectedRoundedY = static_cast<int>(std::round(values.at(0)));
    int expectedRoundedX = static_cast<int>(std::round(values.at(1)));

    int expectedFlooredY = static_cast<int>(std::floor(values.at(0)));
    int expectedFlooredX = static_cast<int>(std::floor(values.at(1)));

    int expectedCeiledY = static_cast<int>(std::ceil(values.at(0)));
    int expectedCeiledX = static_cast<int>(std::ceil(values.at(1)));

    auto rounded = point.template Convert<int, tau::Round>();
    auto floored = point.template Convert<int, tau::Floor>();
    auto ceiled = point.template Convert<int, tau::Ceil>();

    REQUIRE(rounded.y == expectedRoundedY);
    REQUIRE(rounded.x == expectedRoundedX);
    REQUIRE(floored.y == expectedFlooredY);
    REQUIRE(floored.x == expectedFlooredX);
    REQUIRE(ceiled.y == expectedCeiledY);
    REQUIRE(ceiled.x == expectedCeiledX);
}
