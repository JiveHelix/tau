#include <catch2/catch.hpp>

#include <tau/harris.h>
#include <tau/suppression.h>
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


TEST_CASE("Use suppression filter with count = 1", "[harris]")
{
    using Matrix = Eigen::MatrixX<float>;

    Matrix m{
        { 1,  0,  0,  0,  0},
        { 0,  2,  0,  0,  0},
        { 0,  0,  3,  0,  0},
        { 0,  0,  0,  4,  0},
        { 0,  0,  0,  0,  5}};

    tau::Suppression suppression(tau::SuppressionSettings{{3, 1}});
    Matrix filtered = suppression.Filter(m);

    // std::cout << "filtered:\n" << filtered << std::endl;
}


TEST_CASE("Use suppression filter with count = 2", "[harris]")
{
    using Matrix = Eigen::MatrixX<float>;

    Matrix m{
        { 1,  0,  0,  0,  0},
        { 0,  2,  0,  0,  0},
        { 0,  0,  4,  0,  0},
        { 0,  0,  0,  5,  0},
        { 0,  0,  0,  0,  3}};

    tau::Suppression suppression(tau::SuppressionSettings{{3, 2}});
    Matrix filtered = suppression.Filter(m);

    // std::cout << "filtered:\n" << filtered << std::endl;
}


TEST_CASE("Use suppression filter with count = 4", "[harris]")
{
    using Matrix = Eigen::MatrixX<float>;

    Matrix m{
        { 1,  2,  3,  2,  1},
        { 1,  2,  3,  2,  1},
        { 1,  2,  9,  2,  1},
        { 1,  2,  3,  2,  1},
        { 1,  2,  3,  2,  1}};

    tau::Suppression suppression(tau::SuppressionSettings{{3, 1}});
    Matrix filtered = suppression.Filter(m);

    // std::cout << "filtered:\n" << filtered << std::endl;
}
