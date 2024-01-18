/**
 * @author Jive Helix (jivehelix@gmail.com)
 * @copyright 2024 Jive Helix
 * Licensed under the MIT license. See LICENSE file.
 */

#include <catch2/catch.hpp>

#include <cmath>
#include <iostream>
#include <tau/percentile.h>


TEST_CASE("Percentiles from 10 values.", "[percentile]")
{
    auto values =
        tau::Vector<10, double>(1., 2., 3., 4., 5., 6., 7., 8., 9., 10.);

    auto percentiles =
        tau::Vector<2, double>(.1, .9);

    auto results = tau::Percentile(values, percentiles);

    REQUIRE(results.size() == 2);
    REQUIRE(results(0) == 2.);
    REQUIRE(results(1) == 10.);
}


TEST_CASE("Two percentiles from only two values.", "[percentile]")
{
    auto values =
        tau::Vector<2, double>(1., 10.);

    auto percentiles =
        tau::Vector<2, double>(.1, .9);

    auto results = tau::Percentile(values, percentiles);

    REQUIRE(results.size() == 2);
    REQUIRE(results(0) == 1.);
    REQUIRE(results(1) == 10.);
}

