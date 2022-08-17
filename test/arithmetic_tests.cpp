#include <catch2/catch.hpp>

#include <jive/range.h>
#include <jive/testing/generator_limits.h>

#include "tau/arithmetic.h"
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
    using Point = Point2d<T>;

    Point left{{
        static_cast<T>(values.at(0)),
        static_cast<T>(values.at(1))}};

    Point right{{
        static_cast<T>(values.at(2)),
        static_cast<T>(values.at(3))}};

    auto result = Operator{}(left, right); 

    REQUIRE(result.x == static_cast<T>(Operator{}(left.x, right.x)));
    REQUIRE(result.y == static_cast<T>(Operator{}(left.y, right.y)));
}

template<typename T, typename Operator>
void Test3dOperator(const auto &values)
{
    using Point = Point3d<T>;

    Point left{{
        static_cast<T>(values.at(0)),
        static_cast<T>(values.at(1)),
        static_cast<T>(values.at(2))}};

    Point right{{
        static_cast<T>(values.at(3)),
        static_cast<T>(values.at(4)),
        static_cast<T>(values.at(5))}};

    auto result = Operator{}(left, right); 

    REQUIRE(result.x == static_cast<T>(Operator{}(left.x, right.x)));
    REQUIRE(result.y == static_cast<T>(Operator{}(left.y, right.y)));
    REQUIRE(result.z == static_cast<T>(Operator{}(left.z, right.z)));
}

template<typename T, typename Operator>
void Test2dWithScalarOperator(const auto &values)
{
    using Point = Point2d<T>;

    auto scalar = static_cast<T>(values.at(0));

    Point point{{
        static_cast<T>(values.at(1)),
        static_cast<T>(values.at(2))}};

    auto result = Operator{}(point, scalar); 

    REQUIRE(result.x == static_cast<T>(Operator{}(point.x, scalar)));
    REQUIRE(result.y == static_cast<T>(Operator{}(point.y, scalar)));
}

template<typename T, typename Operator>
void Test3dWithScalarOperator(const auto &values)
{
    using Point = Point3d<T>;

    auto scalar = static_cast<T>(values.at(0));

    Point point{{
        static_cast<T>(values.at(1)),
        static_cast<T>(values.at(2)),
        static_cast<T>(values.at(3))}};

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
    "Squared",
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
                3,
                random(Limits::Lowest(), Limits::Max()))));

    Point2d<TestType> value2d{{
        static_cast<TestType>(values.at(0)),
        static_cast<TestType>(values.at(1))}};

    auto result2d = value2d.Squared();

    // Because of the randomly generated inputs, we expect that some operations
    // will overflow. We are testing that the Arithmetic class overflows in the
    // same way as hand-rolled operations.
    auto check2d = static_cast<TestType>(
        value2d.x * value2d.x + value2d.y * value2d.y);

    REQUIRE(result2d == check2d);

    Point3d<TestType> value3d{{
        static_cast<TestType>(values.at(0)),
        static_cast<TestType>(values.at(1)),
        static_cast<TestType>(values.at(2))}};

    auto result3d = value3d.Squared();

    auto check3d = static_cast<TestType>(
        value3d.x * value3d.x
        + value3d.y * value3d.y
        + value3d.z * value3d.z);

    REQUIRE(result3d == check3d);
}


TEMPLATE_TEST_CASE(
    "SquaredDistance",
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

    Point2d<TestType> left2d{{
        static_cast<TestType>(values.at(0)),
        static_cast<TestType>(values.at(1))}};

    Point2d<TestType> right2d{{
        static_cast<TestType>(values.at(2)),
        static_cast<TestType>(values.at(3))}};

    auto result2d = left2d.SquaredDistance(right2d);

    auto check2d = static_cast<TestType>(
        (right2d.x - left2d.x) * (right2d.x - left2d.x)
        + (right2d.y - left2d.y) * (right2d.y - left2d.y));

    REQUIRE(result2d == check2d);

    Point3d<TestType> left3d{{
        static_cast<TestType>(values.at(0)),
        static_cast<TestType>(values.at(1)),
        static_cast<TestType>(values.at(2))}};

    Point3d<TestType> right3d{{
        static_cast<TestType>(values.at(3)),
        static_cast<TestType>(values.at(4)),
        static_cast<TestType>(values.at(5))}};

    auto result3d = left3d.SquaredDistance(right3d);

    auto check3d = static_cast<TestType>(
        (right3d.x - left3d.x) * (right3d.x - left3d.x)
        + (right3d.y - left3d.y) * (right3d.y - left3d.y)
        + (right3d.z - left3d.z) * (right3d.z - left3d.z));

    REQUIRE(result3d == check3d);
}
