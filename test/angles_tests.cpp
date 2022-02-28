/**
 * @author Jive Helix (jivehelix@gmail.com)
 * @copyright 2020 Jive Helix
 * Licensed under the MIT license. See LICENSE file.
 */
#include <catch2/catch.hpp>

#include "tau/angles.h"
#include <cmath>
#include <array>
#include <tuple>
#include <iostream>


// NOLINTNEXTLINE(readability-function-cognitive-complexity)
TEST_CASE("Convert between radians and degrees", "[angles]")
{
    using Angles = tau::Angles<double>;

    const size_t angleCount = 12;

    double radians = -2 * Angles::pi;
    double radiansStep = Angles::pi / static_cast<double>(angleCount);

    double degrees = -Angles::tauDegrees;

    double degreesStep = Angles::piDegrees
        / static_cast<double>(angleCount);

    // Catch's default Approx does nothing when the argument is zero.
    // Give it an explicit margin when we are testing for zero.
    static constexpr double marginNearZero = 1e-13;
    static auto approxZero = Approx(0.0).margin(marginNearZero);

    for (size_t count = 0; count < 2 * angleCount + 1; ++count)
    {
        if (std::abs(degrees) < marginNearZero)
        {
            REQUIRE(tau::ToDegrees(radians) == approxZero);
            REQUIRE(tau::ToRadians(degrees) == approxZero);
        }
        else
        {
            REQUIRE(tau::ToDegrees(radians) == Approx(degrees));
            REQUIRE(tau::ToRadians(degrees) == Approx(radians));
        }

        radians += radiansStep;
        degrees += degreesStep;
    }
}

TEST_CASE("Convert multiple values to degrees", "[Angles]")
{
    // Multple angles (with different types) can be converted in the same
    // variadic call to tau::ToDegrees(...)
    using Angles = tau::Angles<double>;
    static constexpr auto chunkSize = 5;
    static constexpr auto testCount = 100;
    static constexpr double lowerBound = -4 * Angles::pi;
    static constexpr double upperBound = 4 * Angles::pi;

    auto values = GENERATE(
        take(testCount, chunk(chunkSize, random(lowerBound, upperBound))));

    REQUIRE(values.size() == chunkSize);

    double aRadians = values.at(0);
    double bRadians = values.at(1);
    float cRadians = static_cast<float>(values.at(2));
    float dRadians = static_cast<float>(values.at(3));
    double eRadians = values.at(4);

    double aDegrees;
    double bDegrees;
    float cDegrees;
    float dDegrees;
    double eDegrees;

    std::tie(aDegrees, bDegrees, cDegrees, dDegrees, eDegrees) =
        tau::ToDegrees(aRadians, bRadians, cRadians, dRadians, eRadians);

    auto aExpected = Approx(
        Angles::piDegrees * values.at(0) / Angles::pi);

    auto bExpected = Approx(
        Angles::piDegrees * values.at(1) / Angles::pi);

    auto cExpected = Approx(static_cast<float>(
        Angles::piDegrees * values.at(2) / Angles::pi));

    auto dExpected = Approx(static_cast<float>(
        Angles::piDegrees * values.at(3) / Angles::pi));

    auto eExpected = Approx(
        Angles::piDegrees * values.at(4) / Angles::pi);

    REQUIRE(aDegrees == aExpected);
    REQUIRE(bDegrees == bExpected);
    REQUIRE(cDegrees == cExpected);
    REQUIRE(dDegrees == dExpected);
    REQUIRE(eDegrees == eExpected);
}

TEST_CASE("Convert multiple values to radians", "[angles]")
{
    // Multple angles (with different types) can be converted in the same
    // variadic call to tau::ToRadians(...)
    using Angles = tau::Angles<double>;

    static constexpr auto chunkSize = 5;
    static constexpr auto testCount = 100;
    static constexpr double lowerBound = -2 * Angles::tauDegrees;
    static constexpr double upperBound = 2 * Angles::tauDegrees;

    auto values = GENERATE(
        take(testCount, chunk(chunkSize, random(lowerBound, upperBound))));

    REQUIRE(values.size() == 5);

    double aDegrees = values.at(0);
    float bDegrees = static_cast<float>(values.at(1));
    double cDegrees = values.at(2);
    double dDegrees = values.at(3);
    float eDegrees = static_cast<float>(values.at(4));

    double aRadians;
    float bRadians;
    double cRadians;
    double dRadians;
    float eRadians;

    std::tie(aRadians, bRadians, cRadians, dRadians, eRadians) = 
        tau::ToRadians(aDegrees, bDegrees, cDegrees, dDegrees, eDegrees);

    auto aExpected = Approx(
        Angles::pi * values.at(0) / Angles::piDegrees);

    auto bExpected = Approx(static_cast<float>(
        Angles::pi * values.at(1) / Angles::piDegrees));

    auto cExpected = Approx(
        Angles::pi * values.at(2) / Angles::piDegrees);

    auto dExpected = Approx(
        Angles::pi * values.at(3) / Angles::piDegrees);

    auto eExpected = Approx(static_cast<float>(
        Angles::pi * values.at(4) / Angles::piDegrees));

    REQUIRE(aRadians == aExpected);
    REQUIRE(bRadians == bExpected);
    REQUIRE(cRadians == cExpected);
    REQUIRE(dRadians == dExpected);
    REQUIRE(eRadians == eExpected);
}
