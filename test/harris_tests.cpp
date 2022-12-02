#include <catch2/catch.hpp>

#include <tau/harris.h>
#include <tau/gradient.h>



TEST_CASE("Create Harris corner detection class", "[harris]")
{
    using Matrix = Eigen::MatrixX<float>;

    Matrix m{
        { 0,  0,  0,  0,  0, 10, 10, 10, 10, 10},
        { 0,  0,  0,  0,  0, 10, 10, 10, 10, 10},
        { 0,  0,  0,  0,  0, 10, 10, 10, 10, 10},
        { 0,  0,  0,  0,  0, 10, 10, 10, 10, 10},
        { 0,  0,  0,  0,  0, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10,  0,  0,  0,  0,  0},
        {10, 10, 10, 10, 10,  0,  0,  0,  0,  0},
        {10, 10, 10, 10, 10,  0,  0,  0,  0,  0},
        {10, 10, 10, 10, 10,  0,  0,  0,  0,  0},
        {10, 10, 10, 10, 10,  0,  0,  0,  0,  0}};

    m.array() += 10;

    auto differentiate =
        tau::Differentiate<float>(30, 1, tau::DerivativeSize::Size::three);

    auto gradient = tau::Gradient(differentiate, m);
    auto settings = tau::HarrisSettings<float>::Default();
    auto harris = tau::Harris<float>(settings);

    Matrix response = harris.Compute(gradient);

    REQUIRE(response.cols() == m.cols());
    REQUIRE(response.rows() == m.rows());
}
