#include <catch2/catch.hpp>
#include <limits>
#include <random>
#include "tau/eigen.h"
#include <iostream>


TEST_CASE("Change values with Select", "[power_series]")
{
    using namespace tau;

    auto m = Matrix<2, 3, int>(0, 1, 2, 3, 4, 5);

    // tau::Select allows this compact syntax
    // The Eigen way would be:
    //     m = (m.array() <= 2).select(42, m);
    Select(m) <= 2 = 42;

    REQUIRE(m(0, 0) == 42);
    REQUIRE(m(0, 1) == 42);
    REQUIRE(m(0, 2) == 42);
}
