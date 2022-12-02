#include <catch2/catch.hpp>

#include <jive/range.h>
#include <jive/testing/generator_limits.h>

#include "tau/line2d.h"


TEST_CASE("Line2d created from points", "[line2d]")
{
    auto values = GENERATE(
        take(
            16,
            chunk(
                4,
                random(-2000.0, 2000.0))));

    auto origin = tau::Point2d<double>(values.at(0), values.at(1));
    auto endPoint = tau::Point2d<double>(values.at(2), values.at(3));
    auto line = tau::Line2d<double>(origin, endPoint);

    auto expectedDirection = endPoint - origin;

    auto magnitude = std::sqrt(
        std::pow(expectedDirection.x, 2)
        + std::pow(expectedDirection.y, 2));

    expectedDirection /= magnitude;

    REQUIRE(line.vector.x == Approx(expectedDirection.x));
    REQUIRE(line.vector.y == Approx(expectedDirection.y));
}
