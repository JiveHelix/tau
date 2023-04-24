#include <catch2/catch.hpp>

#include <jive/range.h>
#include <jive/testing/generator_limits.h>

#include "tau/arithmetic.h"
#include "tau/random.h"
#include "tau/vector2d.h"
#include "tau/vector3d.h"


TEST_CASE("Compare", "[arithmetic]")
{
    using Point = tau::Point2d<double>;

    Point first(1, 2);
    Point second(3, 4);

    REQUIRE(first < second);

    first.x = 609;
    first.y = 932;
    second.x = 852;
    second.y = 698;

    REQUIRE(first < second);

    first.x = 883.25;
    first.y = 318;
    second.x = 884;
    second.y = 264;

    REQUIRE(first < second);
}


TEST_CASE("Sorted insertion", "[arithmetic]")
{
    using Point = tau::Point2d<double>;
    std::vector<Point> points;

    Point next(884, 264);

    auto insertion = tau::GetUniqueInsertion(points, next);

    REQUIRE(!!insertion);

    REQUIRE(*insertion == end(points));

    points.insert(*insertion, next);


    next.x = 883.25;
    next.y = 318;

    insertion = GetUniqueInsertion(points, next);

    REQUIRE(!!insertion);

    REQUIRE(*insertion == begin(points));

    points.insert(*insertion, next);

    next.x = 882.75;
    next.y = 371.5;

    insertion = GetUniqueInsertion(points, next);

    REQUIRE(!!insertion);

    REQUIRE(*insertion == begin(points));

    points.insert(*insertion, next);

    next.x = 884;
    next.y = 264;

    insertion = GetUniqueInsertion(points, next);

    // This point should already exist.
    REQUIRE(!insertion);

    REQUIRE(std::is_sorted(begin(points), end(points)));
}


TEMPLATE_TEST_CASE(
    "Sort",
    "[arithmetic]",
    double)
{
    auto seed = GENERATE(
        take(16, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    // Default distribution is -1000 to 1000
    tau::UniformRandom<TestType> uniformRandom{seed};

    using Point = tau::Point2d<TestType>;
    std::vector<Point> points;

    for (size_t i = 0; i < 10; ++i)
    {
        points.emplace_back(uniformRandom(), uniformRandom());
    }

    std::sort(begin(points), end(points));

    REQUIRE(std::is_sorted(begin(points), end(points)));
}
