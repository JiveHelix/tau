#include <catch2/catch.hpp>

#include <jive/range.h>
#include <jive/testing/generator_limits.h>

#include "tau/size.h"


TEST_CASE("Size converts to int", "[size]")
{
    auto values = GENERATE(
        take(
            10,
            chunk(
                2,
                random(-100, 100))));

    tau::Size<double> size(values.at(0), values.at(1));

    int expectedRoundedWidth = static_cast<int>(std::round(values.at(0)));
    int expectedRoundedHeight = static_cast<int>(std::round(values.at(1)));

    int expectedFlooredWidth = static_cast<int>(std::floor(values.at(0)));
    int expectedFlooredHeight = static_cast<int>(std::floor(values.at(1)));

    int expectedCeiledWidth = static_cast<int>(std::ceil(values.at(0)));
    int expectedCeiledHeight = static_cast<int>(std::ceil(values.at(1)));

    auto rounded = size.template Convert<int, tau::Round>();
    auto floored = size.template Convert<int, tau::Floor>();
    auto ceiled = size.template Convert<int, tau::Ceil>();

    REQUIRE(rounded.height == expectedRoundedHeight);
    REQUIRE(rounded.width == expectedRoundedWidth);
    REQUIRE(floored.height == expectedFlooredHeight);
    REQUIRE(floored.width == expectedFlooredWidth);
    REQUIRE(ceiled.height == expectedCeiledHeight);
    REQUIRE(ceiled.width == expectedCeiledWidth);
}



TEMPLATE_TEST_CASE(
    "Round-trip succeeds to and from tau::Point2d",
    "[size]",
    int8_t,
    uint8_t,
    int16_t,
    uint16_t,
    int32_t,
    uint32_t,
    int64_t,
    uint64_t,
    float,
    double)
{
    using Limits = GeneratorLimits<TestType>;

    auto values = GENERATE(
        take(
            10,
            chunk(
                2,
                random(Limits::Lowest(), Limits::Max()))));

    tau::Size<TestType> size(
        static_cast<TestType>(values.at(0)),
        static_cast<TestType>(values.at(1)));
    
    auto point = size.ToPoint2d();

    auto roundTrip = tau::Size<TestType>(point);

    REQUIRE(roundTrip == size);
}
