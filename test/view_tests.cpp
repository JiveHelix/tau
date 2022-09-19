#include <catch2/catch.hpp>

#include <jive/range.h>
#include <jive/testing/generator_limits.h>
#include "tau/view.h"
#include "tau/view.h"
#include "random_region.h"


TEMPLATE_TEST_CASE(
    "Overlapping views",
    "[region]",
    int16_t,
    int32_t,
    int64_t,
    float,
    double)
{
    auto seed = GENERATE(
        take(16, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    tau::UniformRandom<TestType> uniformRandom{seed};

    auto sourceSize = MakeInterior<tau::Size, TestType>(
        uniformRandom,
        tau::Point<TestType>{10, 10},
        tau::Size<TestType>{2000, 2000});

    auto viewPosition = MakeInterior<tau::Point, TestType>(
        uniformRandom,
        tau::Point<TestType>{0, 0},
        sourceSize);
    
    tau::Scale<double> scale{{1.0, 1.0}};

    auto view = tau::View(
        tau::Region<TestType>{{viewPosition, sourceSize}},
        sourceSize,
        scale);

    if (viewPosition.x < 0)
    {
        REQUIRE(view.target.topLeft.x == -viewPosition.x);
        REQUIRE(view.source.topLeft.x == 0);
    }
    else
    {
        REQUIRE(view.target.topLeft.x == 0);

        if (view.source.HasArea())
        {
            REQUIRE(view.source.topLeft.x == viewPosition.x);
        }
    }

    if (viewPosition.y < 0)
    {
        REQUIRE(view.target.topLeft.y == -viewPosition.y);
        REQUIRE(view.source.topLeft.y == 0);
    }
    else
    {
        REQUIRE(view.target.topLeft.y == 0);

        if (view.source.HasArea())
        {
            REQUIRE(view.source.topLeft.y == viewPosition.y);
        }
    }

    REQUIRE(view.source.GetBottomRight().x <= Approx(sourceSize.width));
    REQUIRE(view.source.GetBottomRight().y <= Approx(sourceSize.height));
    REQUIRE(view.source.size.GetArea() == Approx(view.target.size.GetArea()));
}


TEMPLATE_TEST_CASE(
    "Overlapping view with negative position",
    "[region]",
    int16_t,
    int32_t,
    int64_t,
    float,
    double)
{
    auto seed = GENERATE(
        take(16, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    tau::UniformRandom<TestType> uniformRandom{seed};

    auto sourceSize = MakeInterior<tau::Size, TestType>(
        uniformRandom,
        tau::Point<TestType>{10, 10},
        tau::Size<TestType>{2000, 2000});

    auto viewPosition = MakeInterior<tau::Point, TestType>(
        uniformRandom,
        (sourceSize / -2).ToPoint(),
        sourceSize / 2);
    
    tau::Scale<double> scale{{1.0, 1.0}};

    auto view = tau::View(
        tau::Region<TestType>{{viewPosition, sourceSize}},
        sourceSize,
        scale);

    if (viewPosition.x < 0)
    {
        REQUIRE(view.target.topLeft.x == -viewPosition.x);
        REQUIRE(view.source.topLeft.x == 0);
    }
    else
    {
        REQUIRE(view.target.topLeft.x == 0);

        if (view.source.HasArea())
        {
            REQUIRE(view.source.topLeft.x == viewPosition.x);
        }
    }

    if (viewPosition.y < 0)
    {
        REQUIRE(view.target.topLeft.y == -viewPosition.y);
        REQUIRE(view.source.topLeft.y == 0);
    }
    else
    {
        REQUIRE(view.target.topLeft.y == 0);

        if (view.source.HasArea())
        {
            REQUIRE(view.source.topLeft.y == viewPosition.y);
        }
    }

    REQUIRE(view.source.GetBottomRight().x <= Approx(sourceSize.width));
    REQUIRE(view.source.GetBottomRight().y <= Approx(sourceSize.height));
    REQUIRE(view.source.size.GetArea() == Approx(view.target.size.GetArea()));
}
