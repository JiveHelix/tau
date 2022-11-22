#include <catch2/catch.hpp>
#include <tau/convolve.h>



TEMPLATE_TEST_CASE(
    "Convolve returns expected results",
    "[convolve]",
    int,
    float,
    double)
{
    using Partial0 = Eigen::Vector<TestType, 3>;
    using Partial1 = Eigen::RowVector<TestType, 3>;

    Partial0 partial0(1, 2, 1);
    Partial1 partial1 = partial0.transpose();

    Eigen::Matrix<TestType, 3, 3> kernel = partial0 * partial1;

    Eigen::Matrix<TestType, 6, 6> input{
        {15, 20, 25, 25, 15, 10},
        {20, 15, 50, 30, 20, 15},
        {20, 50, 55, 60, 30, 20},
        {20, 15, 65, 30, 15, 30},
        {15, 20, 30, 20, 25, 30},
        {20, 25, 15, 20, 10, 15}};

    Eigen::Matrix<int, 6, 6> expected{
        {15, 20, 25, 25, 15, 10},
        {20, 28, 38, 35, 23, 15},
        {20, 35, 48, 43, 28, 20},
        {20, 31, 42, 36, 26, 30},
        {15, 23, 28, 25, 22, 30},
        {20, 25, 15, 20, 10, 15}};

    std::cout << "kernel:\n" << kernel << std::endl;
    std::cout << "convolve:\n" << tau::Convolve(input, kernel) << std::endl;

    // Convolve will normalize the result for us.
    Eigen::Matrix<int, 6, 6> result =
        tau::Normalize(tau::Convolve(input, kernel), kernel)
            .array().round().template cast<int>();

    REQUIRE(result == expected);

    // Results of DoConvolve must be normalized
    Eigen::Matrix<TestType, 6, 6> separable =
        tau::DoConvolve(input, partial0, partial1);

    Eigen::Matrix<int, 6, 6> separableResult =
        tau::Normalize(separable, kernel).array().round().template cast<int>();

    REQUIRE(separableResult.block(1, 1, 4, 4) == expected.block(1, 1, 4, 4));
}
