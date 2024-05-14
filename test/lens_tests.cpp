#include <catch2/catch.hpp>

#include <jive/testing/generator_limits.h>
#include <jive/equal.h>
#include <tau/eigen.h>
#include <tau/random.h>
#include <tau/lens.h>

#ifdef _WIN32
#undef near
#undef far
#endif

TEMPLATE_TEST_CASE(
    "Circle of confusion calculation",
    "[lens]",
    float,
    double)
{
    tau::Lens<TestType> lens
    {
        static_cast<TestType>(5),
        static_cast<TestType>(1.2)
    };

    tau::CircleOfConfusion<TestType> circle(lens, static_cast<TestType>(0.025));

    auto near = static_cast<TestType>(2.84);
    auto far = static_cast<TestType>(20.6);

    static constexpr auto tolerance = static_cast<TestType>(1e-6);
    static constexpr auto expected = static_cast<TestType>(80e-6);

    REQUIRE(jive::Roughly(circle(near), tolerance) == expected);
    REQUIRE(jive::Roughly(circle(far), tolerance) == expected);
}
