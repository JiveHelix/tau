#pragma once

#include <catch2/catch.hpp>

#include "tau/region.h"
#include "tau/random.h"


template<typename T>
tau::Region<T> MakeRandomRegion(tau::UniformRandom<T> &uniformRandom)
{
    using Limits = std::numeric_limits<T>;

    tau::Point2d<T> topLeft;
    tau::Size<T> size;

    // Require generated region to have sides of at least 2.
    // topLeft must leave room for a size of at least 2x2.
    // UniformRandom expects a range of values, so leave at least 3 for the
    // size.
    static constexpr auto minimumSize = 2;
    static constexpr auto maximumTopLeft = minimumSize + 1;

    {
        // Generate topLeft point
        tau::RestoreDistribution restore(uniformRandom);

        if (uniformRandom.GetHigh() > Limits::max() - maximumTopLeft)
        {
            uniformRandom.SetHigh(Limits::max() - maximumTopLeft);
        }

        topLeft = tau::Point2d<T>{uniformRandom(), uniformRandom()};
    }

    {
        // Generate size
        tau::RestoreDistribution restore(uniformRandom);

        // Require generated region to have sides of at least 2.
        uniformRandom.SetLow(minimumSize);

        // Size cannot cause the region to extend beyond the limits of the type.
        uniformRandom.SetHigh(
            std::min<T>(
                uniformRandom.GetHigh(),
                Limits::max()
                    - std::max<T>(0, std::max(topLeft.x, topLeft.y))));

        REQUIRE(static_cast<long>(uniformRandom.GetHigh()) > minimumSize);

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


template<template <typename> typename Pair, typename T>
Pair<T> MakeInterior(
    tau::UniformRandom<T> &uniformRandom,
    const tau::Point2d<T> &topLeft,
    const tau::Size<T> &size)
{
    assert(size.width > 0);
    assert(size.height > 0);

    tau::RestoreDistribution restore(uniformRandom);

    auto low = topLeft.x;
    auto high = CheckAdd(topLeft.x, size.width - 1);

    uniformRandom.SetRange(low, high);

    auto interiorX = uniformRandom();

    REQUIRE(interiorX <= uniformRandom.GetHigh());

    low = topLeft.y;
    high = CheckAdd(topLeft.y, size.height - 1);

    uniformRandom.SetRange(low, high);

    auto interiorY = uniformRandom();

    REQUIRE(interiorY <= uniformRandom.GetHigh());

    return {interiorX, interiorY};
}


template<typename T>
tau::Region<T> MakeIntersectingRegion(
    tau::UniformRandom<T> &uniformRandom,
    const tau::Region<T> &regionToIntersect)
{
    using Limits = std::numeric_limits<T>;

    auto topLeft = regionToIntersect.topLeft;
    auto size = regionToIntersect.size;

    auto interior = MakeInterior<tau::Point2d, T>(uniformRandom, topLeft, size);

    // Reduce size of intersecting region to keep it within the limits of the
    // type.
    if (interior.x + size.width > Limits::max())
    {
        size.width = Limits::max() - interior.x;
    }

    if (interior.y + size.height > Limits::max())
    {
        size.height = Limits::max() - interior.y;
    }

    tau::Region<T> result{{interior, size}};

    REQUIRE(result.topLeft <= regionToIntersect.GetBottomRight());

    REQUIRE(result.GetBottomRight() > result.topLeft);

    return result;
}


template<typename T>
tau::Region<T> MakeInteriorRegion(
    tau::UniformRandom<T> &uniformRandom,
    const tau::Region<T> &exteriorRegion)
{
    REQUIRE(exteriorRegion.GetArea() > 1);

    auto topLeft = exteriorRegion.topLeft;
    auto size = exteriorRegion.size;

    auto interior = MakeInterior<tau::Point2d, T>(uniformRandom, topLeft, size);
    auto maximumSize = tau::Size<T>(exteriorRegion.GetBottomRight() - interior);

    auto interiorSize = MakeInterior<tau::Size, T>(
        uniformRandom,
        {0, 0},
        maximumSize);

    tau::Region<T> result{{interior, interiorSize}};

    if (result.size.GetArea() == 0)
    {
        return MakeInteriorRegion(uniformRandom, exteriorRegion);
    }

    REQUIRE(result.topLeft >= exteriorRegion.topLeft);
    REQUIRE(result.GetBottomRight() <= exteriorRegion.GetBottomRight());

    return result;
}


template<typename T>
tau::Region<T> MakeExteriorRegion(
    tau::UniformRandom<T> &uniformRandom,
    const tau::Region<T> &interiorRegion)
{
    auto rangeSize = tau::Size<T>{1000, 1000};

    auto topLeft = interiorRegion.topLeft - rangeSize.ToPoint2d();

    topLeft.x = max(topLeft.x, 0);
    topLeft.y = max(topLeft.y, 0);

    auto exteriorTopLeft =
        MakeInterior<tau::Point2d, T>(
            uniformRandom,
            topLeft,
            rangeSize - tau::Size<T>{1, 1});

    auto exteriorBottomRight =
        MakeInterior<tau::Point2d, T>(
            uniformRandom,
            interiorRegion.GetBottomRight() + tau::Point2d<T>(1, 1),
            rangeSize);

    return {{
        exteriorTopLeft,
        tau::Size<T>(exteriorBottomRight - exteriorTopLeft)}};
}


template<typename T>
tau::Region<T> MakeNonintersectingRegion(
    tau::UniformRandom<T> &uniformRandom,
    const tau::Region<T> &region)
{
    auto rangeSize = tau::Size<T>{1000, 1000};
    auto topLeft = region.topLeft - rangeSize.ToPoint2d();
    topLeft.x = max(topLeft.x, 0);
    topLeft.y = max(topLeft.y, 0);

    auto actualRangeSize = tau::Size<T>(region.topLeft - topLeft);

    actualRangeSize -= tau::Size<T>{1, 1};

    REQUIRE(actualRangeSize.GetArea() > 0);

    auto bottomRight = MakeInterior<tau::Point2d, T>(
        uniformRandom,
        topLeft + tau::Size<T>{1, 1},
        actualRangeSize);

    REQUIRE(bottomRight < region.topLeft);
}
