#include <catch2/catch.hpp>

#include <jive/range.h>
#include <jive/testing/generator_limits.h>

#include "tau/region.h"
#include "tau/random.h"


template<typename T>
tau::Region<T> MakeRandomRegion(tau::UniformRandom<T> &uniformRandom)
{
    using Limits = std::numeric_limits<T>;

    tau::Point<T> topLeft;
    tau::Size<T> size;

    {
        tau::RestoreDistribution restore(uniformRandom);

        // Require generated region to have sides of at least 2.
        if (uniformRandom.GetHigh() > Limits::max() - 2)
        {
            uniformRandom.SetHigh(Limits::max() - 2);
        }

        topLeft = tau::Point<T>{uniformRandom(), uniformRandom()};
    }

    {
        tau::RestoreDistribution restore(uniformRandom);

        // Require generated region to have sides of at least 2.
        uniformRandom.SetLow(2);

        // Size cannot cause the region to extend beyond the limits of the type.
        uniformRandom.SetHigh(
            std::min<T>(
                uniformRandom.GetHigh(),
                Limits::max()
                    - std::max<T>(0, std::max(topLeft.x, topLeft.y))));

        REQUIRE(static_cast<long>(uniformRandom.GetHigh()) >= 2);

        size = tau::Size<T>{uniformRandom(), uniformRandom()};
    }

    REQUIRE(size.width > 1);
    REQUIRE(size.height > 1);

    tau::Region<T> result{{topLeft, size}};

    REQUIRE(result.GetBottomRight() > result.topLeft);

    return result;
}


template<typename T, typename U>
T CheckSubstract(T left, U right)
{
    if constexpr (std::is_unsigned_v<T>)
    {
        if (left < static_cast<T>(right))
        {
            return 0;
        }
        else
        {
            return static_cast<T>(left - right);
        }
    }
    else
    {
        // signed
        if (left - right <= std::numeric_limits<T>::lowest())
        {
            return std::numeric_limits<T>::lowest();
        }

        return static_cast<T>(left - right);
    }
}


template<typename T, typename U>
T CheckAdd(T left, U right)
{
    if (left + right >= std::numeric_limits<T>::max())
    {
        return std::numeric_limits<T>::max();
    }

    return static_cast<T>(left + right);
}


template<typename T>
tau::Region<T> MakeIntersectingRegion(
    tau::UniformRandom<T> &uniformRandom,
    const tau::Region<T> &regionToIntersect)
{
    using Limits = std::numeric_limits<T>;

    auto topLeft = regionToIntersect.topLeft;
    auto size = regionToIntersect.size;

    tau::RestoreDistribution restore(uniformRandom);

    // Randomly generate an intersecting region.
    uniformRandom.SetRange(
        CheckSubstract(topLeft.x, size.width - 1),
        CheckAdd(topLeft.x, size.width - 1));

    auto intersectingX = uniformRandom();

    REQUIRE(intersectingX <= uniformRandom.GetHigh());

    uniformRandom.SetRange(
        CheckSubstract(topLeft.y, size.height - 1),
        CheckAdd(topLeft.y, size.height - 1));

    auto intersectingY = uniformRandom();

    REQUIRE(intersectingY <= uniformRandom.GetHigh());

    // Reduce size of intersecting region to keep it within the limits of the
    // type.
    if (intersectingX + size.width > Limits::max())
    {
        size.width = Limits::max() - intersectingX;
    }

    if (intersectingY + size.height > Limits::max())
    {
        size.height = Limits::max() - intersectingY;
    }

    tau::Region<T> result{{{intersectingY, intersectingX}, size}};

    REQUIRE(result.topLeft <= regionToIntersect.GetBottomRight());

    REQUIRE(result.GetBottomRight() > result.topLeft);

    return result;
}


TEMPLATE_TEST_CASE(
    "Overlapping regions intersect",
    "[region]",
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

    tau::UniformRandom<TestType> uniformRandom{seed};
    
    auto a = MakeRandomRegion(uniformRandom);
    auto b = MakeIntersectingRegion(uniformRandom, a);

    if (!a.Intersects(b))
    {
        std::cout << "a: " << a << "br: " << a.GetBottomRight() << std::endl;
        std::cout << "b: " << b << "br: " << b.GetBottomRight() << std::endl;
    }

    REQUIRE(a.Intersects(b));
    REQUIRE(b.Intersects(a));
}


