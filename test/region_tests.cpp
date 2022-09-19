#include <catch2/catch.hpp>

#include <jive/range.h>
#include <jive/testing/generator_limits.h>

#include "random_region.h"


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


