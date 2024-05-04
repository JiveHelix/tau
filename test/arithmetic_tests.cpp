#include <catch2/catch.hpp>

#include <jive/range.h>
#include <jive/testing/generator_limits.h>

#include "tau/arithmetic.h"
#include "tau/random.h"
#include "tau/vector2d.h"
#include "tau/vector3d.h"


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
void Test2dOperator(const auto &lefts, const auto &rights)
{
    using Point = tau::Point2d<T>;

    Point left(
        static_cast<T>(lefts.at(0)),
        static_cast<T>(lefts.at(1)));

    Point right(
        static_cast<T>(rights.at(0)),
        static_cast<T>(rights.at(1)));

    auto result = Operator{}(left, right);

    REQUIRE(result.x == static_cast<T>(Operator{}(left.x, right.x)));
    REQUIRE(result.y == static_cast<T>(Operator{}(left.y, right.y)));
}

template<typename T, typename Operator>
void Test3dOperator(const auto &lefts, const auto &rights)
{
    using Point = tau::Point3d<T>;

    Point left(
        static_cast<T>(lefts.at(0)),
        static_cast<T>(lefts.at(1)),
        static_cast<T>(lefts.at(2)));

    Point right(
        static_cast<T>(rights.at(0)),
        static_cast<T>(rights.at(1)),
        static_cast<T>(rights.at(2)));

    auto result = Operator{}(left, right);

    REQUIRE(result.x == static_cast<T>(Operator{}(left.x, right.x)));
    REQUIRE(result.y == static_cast<T>(Operator{}(left.y, right.y)));
    REQUIRE(result.z == static_cast<T>(Operator{}(left.z, right.z)));
}

template<typename T, typename Operator>
void Test2dWithScalarOperator(const auto &values, auto scalar)
{
    using Point = tau::Point2d<T>;

    Point point(
        static_cast<T>(values.at(0)),
        static_cast<T>(values.at(1)));

    auto result = Operator{}(point, scalar);

    REQUIRE(result.x == static_cast<T>(Operator{}(point.x, scalar)));
    REQUIRE(result.y == static_cast<T>(Operator{}(point.y, scalar)));
}

template<typename T, typename Operator>
void Test3dWithScalarOperator(const auto &values, auto scalar)
{
    using Point = tau::Point3d<T>;

    Point point(
        static_cast<T>(values.at(0)),
        static_cast<T>(values.at(1)),
        static_cast<T>(values.at(2)));

    auto result = Operator{}(point, scalar);

    REQUIRE(result.x == static_cast<T>(Operator{}(point.x, scalar)));
    REQUIRE(result.y == static_cast<T>(Operator{}(point.y, scalar)));
    REQUIRE(result.z == static_cast<T>(Operator{}(point.z, scalar)));
}


template<typename T>
struct AddLimits
{
    using Limits = tau::DefaultRange<T>;
    static constexpr auto lowest = static_cast<int64_t>(Limits::low);
    static constexpr auto highest = static_cast<int64_t>(Limits::high);

    // value + x >= lowest
    // value + x <= highest
    // x >= lowest - value
    // x <= highest - value

    static int64_t GetLowest(T value)
    {
        auto value_ = static_cast<int64_t>(value);
        return std::max(lowest, lowest - value_);
    }

    static int64_t GetHighest(T value)
    {
        auto value_ = static_cast<int64_t>(value);
        return std::min(highest, highest - value_);
    }
};


template<typename T>
class SubtractLimits
{
public:
    using Limits = tau::DefaultRange<T>;
    static constexpr auto lowest = static_cast<int64_t>(Limits::low);
    static constexpr auto highest = static_cast<int64_t>(Limits::high);

    // value - x >= lowest
    // value - x <= highest
    // x <= value - lowest
    // x >= value - highest

    static int64_t GetLowest(T value)
    {
        auto value_ = static_cast<int64_t>(value);
        return std::max(lowest, value_ - highest);
    }

    static int64_t GetHighest(T value)
    {
        auto value_ = static_cast<int64_t>(value);
        return std::min(highest, value_ - lowest);
    }
};


template<typename T>
class MultiplyLimits
{
public:
    using Limits = tau::DefaultRange<T>;
    static constexpr auto lowest = static_cast<int64_t>(Limits::low);
    static constexpr auto highest = static_cast<int64_t>(Limits::high);

    // value * x >= lowest
    // value * x <= highest
    // x >= lowest / value;
    // x <= highest / value;

    static int64_t GetLowest(T value)
    {
        auto value_ = static_cast<int64_t>(value);
        return lowest / value_;
    }

    static int64_t GetHighest(T value)
    {
        auto value_ = static_cast<int64_t>(value);
        return highest / value_;
    }
};


template<typename T>
class DivideLimits
{
public:
    using Limits = tau::DefaultRange<T>;
    static constexpr auto lowest = static_cast<int64_t>(Limits::low);
    static constexpr auto highest = static_cast<int64_t>(Limits::high);

    static int64_t GetLowest(T)
    {
        return lowest;
    }

    static int64_t GetHighest(T)
    {
        return highest;
    }
};


template
<
    typename T,
    template<typename> typename LimitFunctor,
    template<typename> typename Filter = tau::DefaultFilter
>
struct TestInputs
{
    using Limits = tau::DefaultRange<T>;
    static constexpr auto lowest = static_cast<int64_t>(Limits::low);
    static constexpr auto highest = static_cast<int64_t>(Limits::high);

    tau::UniformRandom<T, Filter> uniformRandom;

    int64_t scalarLow = static_cast<int64_t>(Limits::low);
    int64_t scalarHigh = static_cast<int64_t>(Limits::high);

    std::vector<T> lefts;
    std::vector<T> rights;
    std::vector<T> scalars;

    TestInputs(tau::Seed seed)
        :
        uniformRandom(seed)
    {
        this->uniformRandom.SetRange(Limits::low + 1, Limits::high - 1);

        this->lefts.push_back(uniformRandom());
        this->lefts.push_back(uniformRandom());
        this->lefts.push_back(uniformRandom());

        for (auto &left: this->lefts)
        {
            if constexpr (std::is_integral_v<T>)
            {
                if constexpr (std::is_signed_v<T>)
                {
                    auto lowestRight = LimitFunctor<T>::GetLowest(left);
                    auto highestRight = LimitFunctor<T>::GetHighest(left);

                    if (lowestRight > highestRight)
                    {
                        std::swap(lowestRight, highestRight);
                    }

                    uniformRandom.SetRange(
                        static_cast<T>(lowestRight),
                        static_cast<T>(highestRight));

                    this->scalarLow = std::max(this->scalarLow, lowestRight);
                    this->scalarHigh = std::min(this->scalarHigh, highestRight);
                }
                else
                {
                    auto highestRight = LimitFunctor<T>::GetHighest(left);

                    uniformRandom.SetRange(
                        0,
                        static_cast<T>(highestRight));

                    this->scalarHigh =
                        std::min(
                            this->scalarHigh,
                            static_cast<int64_t>(highestRight));
                }
            }

            this->rights.push_back(uniformRandom());
        }

        uniformRandom.SetRange(
            static_cast<T>(this->scalarLow),
            static_cast<T>(this->scalarHigh));

        this->scalars.push_back(uniformRandom());
        this->scalars.push_back(uniformRandom());
    }

};


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
    auto seed = GENERATE(
        take(16, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    // Create three pairs of values that can add without overflow.
    // Default distribution is -1000 to 1000, or the lowest and highest values
    // representable by TestType.
    auto inputs = TestInputs<TestType, AddLimits>(seed);

    Test2dOperator<TestType, Add>(inputs.lefts, inputs.rights);
    Test3dOperator<TestType, Add>(inputs.lefts, inputs.rights);

    Test2dWithScalarOperator<TestType, Add>(
        inputs.lefts,
        inputs.scalars.at(0));

    Test3dWithScalarOperator<TestType, Add>(
        inputs.lefts,
        inputs.scalars.at(1));
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
    auto seed = GENERATE(
        take(16, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    // Create three pairs of values that can add without overflow.
    // Default distribution is -1000 to 1000, or the lowest and highest values
    // representable by TestType.
    auto inputs = TestInputs<TestType, SubtractLimits>(seed);

    Test2dOperator<TestType, Subtract>(inputs.lefts, inputs.rights);
    Test3dOperator<TestType, Subtract>(inputs.lefts, inputs.rights);

    Test2dWithScalarOperator<TestType, Subtract>(
        inputs.lefts,
        inputs.scalars.at(0));

    Test3dWithScalarOperator<TestType, Subtract>(
        inputs.lefts,
        inputs.scalars.at(1));
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
    auto seed = GENERATE(
        take(16, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    // Create three pairs of values that can add without overflow.
    // Default distribution is -1000 to 1000, or the lowest and highest values
    // representable by TestType.
    auto inputs = TestInputs<TestType, MultiplyLimits>(seed);

    Test2dOperator<TestType, Multiply>(inputs.lefts, inputs.rights);
    Test3dOperator<TestType, Multiply>(inputs.lefts, inputs.rights);

    Test2dWithScalarOperator<TestType, Multiply>(
        inputs.lefts,
        inputs.scalars.at(0));

    Test3dWithScalarOperator<TestType, Multiply>(
        inputs.lefts,
        inputs.scalars.at(1));
}


template<typename T>
struct DivideFilter
{
    bool operator() (T value) const
    {
        return value != 0;
    }
};


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
    auto seed = GENERATE(
        take(16, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    // Create three pairs of values that can add without overflow.
    // Default distribution is -1000 to 1000, or the lowest and highest values
    // representable by TestType.
    auto inputs = TestInputs<TestType, DivideLimits, DivideFilter>(seed);

    Test2dOperator<TestType, Divide>(inputs.lefts, inputs.rights);
    Test3dOperator<TestType, Divide>(inputs.lefts, inputs.rights);

    Test2dWithScalarOperator<TestType, Divide>(
        inputs.lefts,
        inputs.scalars.at(0));

    Test3dWithScalarOperator<TestType, Divide>(
        inputs.lefts,
        inputs.scalars.at(1));
}
#if 0


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

#endif


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
            // Function-style cast to TestType to silence warning.
            uniformRandom.SetRange(TestType(-maximum), maximum);
        }
        else
        {
            uniformRandom.SetRange(0, maximum);
        }
    }

    tau::Point2d<TestType> value2d(uniformRandom(), uniformRandom());

    auto result2d = value2d.SquaredSum();

    auto check2d = static_cast<TestType>(
        value2d.x * value2d.x + value2d.y * value2d.y);

    REQUIRE(result2d == Approx(check2d));
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
            // Cast to TestType to silence warning.
            uniformRandom.SetRange(TestType(-maximum), maximum);
        }
        else
        {
            uniformRandom.SetRange(0, maximum);
        }
    }

    tau::Point3d<TestType> value3d(
        uniformRandom(),
        uniformRandom(),
        uniformRandom());

    auto result3d = value3d.SquaredSum();

    auto check3d = static_cast<TestType>(
        value3d.x * value3d.x
        + value3d.y * value3d.y
        + value3d.z * value3d.z);

    REQUIRE(result3d == Approx(check3d));
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

    tau::Point2d<TestType> left2d;
    tau::Point2d<TestType> right2d;

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

            // Cast to TestType to silence warning.
            uniformRandom.SetRange(TestType(-maximum), maximum);

            left2d =
                tau::Point2d<TestType>(uniformRandom(), uniformRandom());

            right2d =
                tau::Point2d<TestType>(uniformRandom(), uniformRandom());
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

            left2d =
                tau::Point2d<TestType>(uniformRandom(), uniformRandom());

            uniformRandom.SetRange(maximum / 2, maximum);

            right2d =
                tau::Point2d<TestType>(uniformRandom(), uniformRandom());
        }
    }
    else
    {
        left2d = tau::Point2d<TestType>(uniformRandom(), uniformRandom());
        right2d = tau::Point2d<TestType>(uniformRandom(), uniformRandom());
    }

    auto result2d = (right2d - left2d).SquaredSum();

    auto check2d = static_cast<TestType>(
        (right2d.x - left2d.x) * (right2d.x - left2d.x)
        + (right2d.y - left2d.y) * (right2d.y - left2d.y));

    REQUIRE(result2d == Approx(check2d));
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

    tau::Point3d<TestType> left3d;
    tau::Point3d<TestType> right3d;

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

            // Cast to TestType to silence warning.
            // Adding the negative sign caused a conversion to 'int'.
            uniformRandom.SetRange(TestType(-maximum), maximum);

            left3d = tau::Point3d<TestType>(
                uniformRandom(),
                uniformRandom(),
                uniformRandom());

            right3d = tau::Point3d<TestType>(
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

            left3d = tau::Point3d<TestType>(
                uniformRandom(),
                uniformRandom(),
                uniformRandom());

            uniformRandom.SetRange(maximum / 2, maximum);

            right3d = tau::Point3d<TestType>(
                uniformRandom(),
                uniformRandom(),
                uniformRandom());
        }
    }
    else
    {
        left3d = tau::Point3d<TestType>(
            uniformRandom(),
            uniformRandom(),
            uniformRandom());

        right3d = tau::Point3d<TestType>(
            uniformRandom(),
            uniformRandom(),
            uniformRandom());
    }

    auto result3d = (right3d - left3d).SquaredSum();

    auto check3d = static_cast<TestType>(
        (right3d.x - left3d.x) * (right3d.x - left3d.x)
        + (right3d.y - left3d.y) * (right3d.y - left3d.y)
        + (right3d.z - left3d.z) * (right3d.z - left3d.z));

    REQUIRE(result3d == Approx(check3d));
}
