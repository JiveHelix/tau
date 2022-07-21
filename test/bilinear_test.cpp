#include <catch2/catch.hpp>
#include "tau/bilinear.h"


TEST_CASE("Bilinear resampling larger has correct values.", "[bilinear]")
{
    using namespace tau;

    auto m = Matrix<3, 4, int>(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11);

    Eigen::Matrix<int, 4, 6> result = tau::Bilinear(m, 4, 6);

    auto expected = Matrix<4, 6, int>(
        0,  1,  1,  2,  2,  3,
        3,  3,  4,  4,  5,  6,
        5,  6,  7,  7,  8,  8,
        8,  9,  9, 10, 10, 11);

    REQUIRE(result == expected);
}
