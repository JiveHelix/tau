#include <catch2/catch.hpp>

#include <jive/range.h>
#include <jive/equal.h>
#include <jive/testing/generator_limits.h>
#include <iomanip>

#include "tau/vector2d.h"
#include "tau/scale.h"


static constexpr double tolerance = 1e-4;


TEST_CASE("Point2d converts to int", "[vector2d]")
{
    auto values = GENERATE(
        take(
            10,
            chunk(
                2,
                random(-100, 100))));

    tau::Point2d<double> point(values.at(0), values.at(1));

    int expectedRoundedX = static_cast<int>(std::round(values.at(0)));
    int expectedRoundedY = static_cast<int>(std::round(values.at(1)));

    int expectedFlooredX = static_cast<int>(std::floor(values.at(0)));
    int expectedFlooredY = static_cast<int>(std::floor(values.at(1)));

    int expectedCeiledX = static_cast<int>(std::ceil(values.at(0)));
    int expectedCeiledY = static_cast<int>(std::ceil(values.at(1)));

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


TEMPLATE_TEST_CASE(
    "Integer Points are floored when multiplied by tau::Scale",
    "[vector2d]",
    int8_t,
    uint8_t,
    int16_t,
    uint16_t,
    int32_t,
    uint32_t,
    int64_t,
    uint64_t)
{
    using Limits = GeneratorLimits<TestType>;
    using Target = typename Limits::Target;

    auto values = GENERATE(
        take(
            8,
            chunk(
                2,
                random(
                    static_cast<Target>(
                        static_cast<double>(Limits::Lowest()) / 1.51),
                    static_cast<Target>(
                        static_cast<double>(Limits::Max()) / 1.51)))));

    auto scalars =
        GENERATE(chunk(2, take(3, random(0.0, 1.5))));

    auto point = tau::Point2d<TestType>{
        static_cast<TestType>(values.at(0)),
        static_cast<TestType>(values.at(1))};

    auto scale = tau::Scale<double>(scalars.at(0), scalars.at(1));

    auto expectedX = static_cast<TestType>(
        std::floor(scale.horizontal * static_cast<double>(point.x)));

    auto expectedY = static_cast<TestType>(
        std::floor(scale.vertical * static_cast<double>(point.y)));

    auto result = point * scale;

    REQUIRE(result.x == expectedX);
    REQUIRE(result.y == expectedY);
}


TEST_CASE("Point2d can be multiplied by a scalar", "[vector2d]")
{
    auto values = GENERATE(take(8, chunk(3, random(0.0, 100.0))));

    tau::Point2d<double> point{values.at(0), values.at(1)};

    auto result = point * values.at(2);

    REQUIRE(result.x == point.x * values.at(2));
    REQUIRE(result.y == point.y * values.at(2));
}


#if 0
TEST_CASE("Strict comparison", "[vector2d]")
{
    tau::Point2d<int> p(2, -3);
    tau::Point2d<int> q(0, 0);

    REQUIRE(!(p > q));
    REQUIRE(!(p < q));
    REQUIRE((p != q));
}
#endif


TEST_CASE("Rotate vector 90 degrees", "[vector2d]")
{
    tau::Vector2d<double> v(1, 0);
    auto r = v.Rotate(90);

    REQUIRE(jive::Roughly(r.x, tolerance) == 0);
    REQUIRE(jive::Roughly(r.y, tolerance) == 1);

    r = r.Rotate(90);
    REQUIRE(jive::Roughly(r.x, tolerance) == -1);
    REQUIRE(jive::Roughly(r.y, tolerance) == 0);

    r = r.Rotate(90);
    REQUIRE(jive::Roughly(r.x, tolerance) == 0);
    REQUIRE(jive::Roughly(r.y, tolerance) == -1);

    r = r.Rotate(90);
    REQUIRE(jive::Roughly(r.x, tolerance) == 1);
    REQUIRE(jive::Roughly(r.y, tolerance) == 0);
}


TEST_CASE("Rotate vector", "[vector2d]")
{
    // Generate values with a magnitude greater than 0.
    auto angle_deg = GENERATE(take(16, random(-180.0, 180.0)));

    tau::Vector2d<double> v(1, 0);
    auto r = v.Rotate(angle_deg);

    REQUIRE(jive::Roughly(v.GetAngle(), tolerance) == 0);
    REQUIRE(jive::Roughly(r.GetAngle(), tolerance) == angle_deg);
}


TEST_CASE("Compute distance between points", "[vector2d]")
{
    tau::Point2d<float> start(1, 1);
    tau::Point2d<float> end(4, 5);
    REQUIRE(start.Distance(end) == Approx(5.0f));
}
