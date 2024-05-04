#include <catch2/catch.hpp>
#include <tau/row_convolve.h>


TEMPLATE_TEST_CASE(
    "RowConvolve returns expected results",
    "[convolve]",
    int,
    float,
    double)
{
    Eigen::RowVector<TestType, 3> kernel{{0, 1, 2}};
    Eigen::RowVector<TestType, 10> signal{{0, 1, 2, 3, 4, 5, 6, 7, 8, 9}};

    Eigen::RowVector<TestType, 12> expected{
        {0, 0, 1, 4, 7, 10, 13, 16, 19, 22, 25, 18}};

    Eigen::RowVector<TestType, 12> expectedWithReflect{
        {5, 2, 1, 4, 7, 10, 13, 16, 19, 22, 25, 26}};

    auto result = tau::RowConvolve(signal, kernel, false);
    auto withReflect = tau::RowConvolve(signal, kernel, true);

    REQUIRE(result == expected);
    REQUIRE(withReflect == expectedWithReflect);
}
