#include <catch2/catch.hpp>

#include <jive/range.h>
#include <jive/testing/generator_limits.h>

#include "tau/arithmetic.h"
#include "tau/random.h"
#include "point.h"


struct Add
{
    auto operator()(auto left, auto right) { return left + right; }
};

struct Subtract
{
    auto operator()(auto left, auto right) { return left - right; }
};

struct Multiply
{
    auto operator()(auto left, auto right) { return left * right; }
};

struct Divide
{
    auto operator()(auto left, auto right) { return left / right; }
};


template<typename T, typename Operator>
void Test2dOperator(const auto &values)
{
    using Point = tautest::Point2d<T>;

    Point left(
        static_cast<T>(values.at(0)),
        static_cast<T>(values.at(1)));

    Point right(
        static_cast<T>(values.at(2)),
        static_cast<T>(values.at(3)));

    auto result = Operator{}(left, right);

    REQUIRE(result.x == static_cast<T>(Operator{}(left.x, right.x)));
    REQUIRE(result.y == static_cast<T>(Operator{}(left.y, right.y)));
}

template<typename T, typename Operator>
void Test3dOperator(const auto &values)
{
    using Point = tautest::Point3d<T>;

    Point left(
        static_cast<T>(values.at(0)),
        static_cast<T>(values.at(1)),
        static_cast<T>(values.at(2)));

    Point right(
        static_cast<T>(values.at(3)),
        static_cast<T>(values.at(4)),
        static_cast<T>(values.at(5)));

    auto result = Operator{}(left, right); 

    REQUIRE(result.x == static_cast<T>(Operator{}(left.x, right.x)));
    REQUIRE(result.y == static_cast<T>(Operator{}(left.y, right.y)));
    REQUIRE(result.z == static_cast<T>(Operator{}(left.z, right.z)));
}

template<typename T, typename Operator>
void Test2dWithScalarOperator(const auto &values)
{
    using Point = tautest::Point2d<T>;

    auto scalar = static_cast<T>(values.at(0));

    Point point(
        static_cast<T>(values.at(1)),
        static_cast<T>(values.at(2)));

    auto result = Operator{}(point, scalar); 

    REQUIRE(result.x == static_cast<T>(Operator{}(point.x, scalar)));
    REQUIRE(result.y == static_cast<T>(Operator{}(point.y, scalar)));
}

template<typename T, typename Operator>
void Test3dWithScalarOperator(const auto &values)
{
    using Point = tautest::Point3d<T>;

    auto scalar = static_cast<T>(values.at(0));

    Point point(
        static_cast<T>(values.at(1)),
        static_cast<T>(values.at(2)),
        static_cast<T>(values.at(3)));

    auto result = Operator{}(point, scalar); 

    REQUIRE(result.x == static_cast<T>(Operator{}(point.x, scalar)));
    REQUIRE(result.y == static_cast<T>(Operator{}(point.y, scalar)));
    REQUIRE(result.z == static_cast<T>(Operator{}(point.z, scalar)));
}


TEMPLATE_TEST_CASE(
    "Add",
    "[arithmetic]",
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
                6,
                random(Limits::Lowest(), Limits::Max()))));

    Test2dOperator<TestType, Add>(values);
    Test3dOperator<TestType, Add>(values);
    Test2dWithScalarOperator<TestType, Add>(values);
    Test3dWithScalarOperator<TestType, Add>(values);
}


TEMPLATE_TEST_CASE(
    "Subtract",
    "[arithmetic]",
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
                6,
                random(Limits::Lowest(), Limits::Max()))));

    Test2dOperator<TestType, Subtract>(values);
    Test3dOperator<TestType, Subtract>(values);
    Test2dWithScalarOperator<TestType, Subtract>(values);
    Test3dWithScalarOperator<TestType, Subtract>(values);
}


TEMPLATE_TEST_CASE(
    "Multiply",
    "[arithmetic]",
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
                6,
                random(Limits::Lowest(), Limits::Max()))));

    Test2dOperator<TestType, Multiply>(values);
    Test3dOperator<TestType, Multiply>(values);
    Test2dWithScalarOperator<TestType, Multiply>(values);
    Test3dWithScalarOperator<TestType, Multiply>(values);
}


TEMPLATE_TEST_CASE(
    "Divide",
    "[arithmetic]",
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
                6,
                filter(
                    [] (auto v) { return v != static_cast<decltype(v)>(0); },
                    random(Limits::Lowest(), Limits::Max())))));

    Test2dOperator<TestType, Divide>(values);
    Test3dOperator<TestType, Divide>(values);
    Test2dWithScalarOperator<TestType, Divide>(values);
    Test3dWithScalarOperator<TestType, Divide>(values);
}


TEMPLATE_TEST_CASE(
    "Squared 2d",
    "[arithmetic]",
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
    auto seed = GENERATE(
        take(16, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    // Default distribution is -1000 to 1000
    tau::UniformRandom<TestType> uniformRandom{seed};

    if constexpr (std::is_integral_v<TestType>)
    {
        double limit = std::min(
            static_cast<double>(std::numeric_limits<TestType>::max()),
            10.0e6);

        // Ensure that the sum of squares fits in the TestType.
        auto maximum =
            static_cast<TestType>(std::floor(std::sqrt(limit / 2.0)));

        if constexpr (std::is_signed_v<TestType>)
        {
            uniformRandom.SetRange(-maximum, maximum);
        }
        else
        {
            uniformRandom.SetRange(0, maximum);
        }
    }

    tautest::Point2d<TestType> value2d(uniformRandom(), uniformRandom());

    auto result2d = value2d.SquaredSum();

    auto check2d = static_cast<TestType>(
        value2d.x * value2d.x + value2d.y * value2d.y);

    REQUIRE(result2d == check2d);
}

TEMPLATE_TEST_CASE(
    "Squared 3d",
    "[arithmetic]",
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
    auto seed = GENERATE(
        take(16, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    // Default distribution is -1000 to 1000
    tau::UniformRandom<TestType> uniformRandom{seed};

    if constexpr (std::is_integral_v<TestType>)
    {
        double limit = std::min(
            static_cast<double>(std::numeric_limits<TestType>::max()),
            10.0e6);

        // Ensure that the sum of squares fits in the TestType.
        auto maximum =
            static_cast<TestType>(std::floor(std::sqrt(limit / 3.0)));

        if constexpr (std::is_signed_v<TestType>)
        {
            uniformRandom.SetRange(-maximum, maximum);
        }
        else
        {
            uniformRandom.SetRange(0, maximum);
        }
    }

    tautest::Point3d<TestType> value3d(
        uniformRandom(),
        uniformRandom(),
        uniformRandom());

    auto result3d = value3d.SquaredSum();

    auto check3d = static_cast<TestType>(
        value3d.x * value3d.x
        + value3d.y * value3d.y
        + value3d.z * value3d.z);

    REQUIRE(result3d == check3d);
}


TEMPLATE_TEST_CASE(
    "SquaredDistance 2d",
    "[arithmetic]",
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
    auto seed = GENERATE(
        take(16, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    // Default distribution is -1000 to 1000
    tau::UniformRandom<TestType> uniformRandom{seed};

    tautest::Point2d<TestType> left2d;
    tautest::Point2d<TestType> right2d;

    if constexpr (std::is_integral_v<TestType>)
    {
        // Ensure that the sum of squares fits in the TestType.
        double limit = std::min(
            static_cast<double>(std::numeric_limits<TestType>::max()),
            10.0e6);

        if constexpr (std::is_signed_v<TestType>)
        {
            // A pair of values with opposite signs could subtract to twice
            // their magnitude. Dividing the range by 4 ensures no combination
            // will overflow.
            auto maximum = 
                static_cast<TestType>(std::floor(std::sqrt(limit / 2.0) / 2.0));

            uniformRandom.SetRange(-maximum, maximum);
            left2d = tautest::Point2d<TestType>(uniformRandom(), uniformRandom());
            right2d = tautest::Point2d<TestType>(uniformRandom(), uniformRandom());
        }
        else
        {
            // The TestType is unsigned, and so the difference of values can
            // only get smaller. Ensuring both values are less than half the
            // range when squared ensures no overflow.
            auto maximum =
                static_cast<TestType>(std::floor(std::sqrt(limit / 2.0)));

            // For unsigned comparisons, right2d must be greater than left2d to
            // avoid overflow.
            uniformRandom.SetRange(0, maximum / 2);
            left2d = tautest::Point2d<TestType>(uniformRandom(), uniformRandom());

            uniformRandom.SetRange(maximum / 2, maximum);
            right2d = tautest::Point2d<TestType>(uniformRandom(), uniformRandom());
        }
    }
    else
    {
        left2d = tautest::Point2d<TestType>(uniformRandom(), uniformRandom());
        right2d = tautest::Point2d<TestType>(uniformRandom(), uniformRandom());
    }

    auto result2d = (right2d - left2d).SquaredSum();

    auto check2d = static_cast<TestType>(
        (right2d.x - left2d.x) * (right2d.x - left2d.x)
        + (right2d.y - left2d.y) * (right2d.y - left2d.y));

    REQUIRE(result2d == check2d);
}


TEMPLATE_TEST_CASE(
    "SquaredDistance 3d",
    "[arithmetic]",
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
    auto seed = GENERATE(
        take(16, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    // Default distribution is -1000 to 1000
    tau::UniformRandom<TestType> uniformRandom{seed};

    tautest::Point3d<TestType> left3d;
    tautest::Point3d<TestType> right3d;

    if constexpr (std::is_integral_v<TestType>)
    {
        // Ensure that the sum of squares fits in the TestType.
        double limit = std::min(
            static_cast<double>(std::numeric_limits<TestType>::max()),
            10.0e6);

        if constexpr (std::is_signed_v<TestType>)
        {
            // A pair of values with opposite signs could subtract to twice
            // their magnitude. Because there are three such pairings,
            // divide the range by 6 to ensure no combination will overflow.
            auto maximum = 
                static_cast<TestType>(std::floor(std::sqrt(limit / 3.0) / 3.0));

            uniformRandom.SetRange(-maximum, maximum);

            left3d = tautest::Point3d<TestType>(
                uniformRandom(),
                uniformRandom(),
                uniformRandom());

            right3d = tautest::Point3d<TestType>(
                uniformRandom(),
                uniformRandom(),
                uniformRandom());
        }
        else
        {
            // The TestType is unsigned, and so the difference of values can
            // only get smaller. Require both values to be less than 1/3 the
            // range when squared to ensure no overflow.
            auto maximum =
                static_cast<TestType>(std::floor(std::sqrt(limit / 3.0)));

            uniformRandom.SetRange(0, maximum);

            // For unsigned comparisons, right2d must be greater than left2d to
            // avoid overflow.
            uniformRandom.SetRange(0, maximum / 2);

            left3d = tautest::Point3d<TestType>(
                uniformRandom(),
                uniformRandom(),
                uniformRandom());

            uniformRandom.SetRange(maximum / 2, maximum);

            right3d = tautest::Point3d<TestType>(
                uniformRandom(),
                uniformRandom(),
                uniformRandom());
        }
    }
    else
    {
        left3d = tautest::Point3d<TestType>(
            uniformRandom(),
            uniformRandom(),
            uniformRandom());

        right3d = tautest::Point3d<TestType>(
            uniformRandom(),
            uniformRandom(),
            uniformRandom());
    }

    auto result3d = (right3d - left3d).SquaredSum();

    auto check3d = static_cast<TestType>(
        (right3d.x - left3d.x) * (right3d.x - left3d.x)
        + (right3d.y - left3d.y) * (right3d.y - left3d.y)
        + (right3d.z - left3d.z) * (right3d.z - left3d.z));

    REQUIRE(result3d == check3d);
}
