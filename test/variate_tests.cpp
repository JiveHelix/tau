/**
 * @author Jive Helix (jivehelix@gmail.com)
 * @copyright 2020 Jive Helix
 * Licensed under the MIT license. See LICENSE file.
 */
#include <catch2/catch.hpp>

#include <cmath>
#include <array>
#include <tuple>
#include <iostream>

#include <jive/equal.h>
#include <tau/variate.h>
#include <tau/random.h>


// NOLINTNEXTLINE(readability-function-cognitive-complexity)
TEST_CASE("Multiply variate by scalar", "[variate]")
{
    tau::Variance<double> variance(100, 36);

    std::cout << "variance: " << variance << std::endl;

    variance *= 2;

    std::cout << "variance * 2: " << variance << std::endl;

    REQUIRE(variance.value == 200);
    REQUIRE(variance.variance == 144);
}


TEMPLATE_TEST_CASE(
    "Variance divide",
    "[variance]",
    float,
    double)
{
    auto seed = GENERATE(
        take(32, random(tau::SeedLimits::min(), tau::SeedLimits::max())));

    tau::Variance<TestType> first;
    tau::Variance<TestType> second;
    tau::UniformRandom<TestType> uniformRandom{seed, -1000, 1000};

    first.value = uniformRandom();
    second.value = uniformRandom();

    uniformRandom.SetRange(-100, 100);

    first.variance = uniformRandom();
    second.variance = uniformRandom();

    auto divided = first / second;
    auto invertFirst = first * second.Power(-1);

    REQUIRE(Approx(divided.value) == invertFirst.value);
    REQUIRE(Approx(divided.variance) == invertFirst.variance);
}
