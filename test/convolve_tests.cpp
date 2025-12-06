#include <catch2/catch.hpp>

// #define TAU_CONVOLVE_TRANSPOSE_MAJOR
#include <tau/convolve.h>
#include <tau/view.h>
#include <tau/margins.h>


TEMPLATE_TEST_CASE(
    "Convolve2d returns expected results",
    "[convolve]",
    int,
    float,
    double)
{
    using ColumnKernel = Eigen::Vector<TestType, 3>;
    using RowKernel = Eigen::RowVector<TestType, 3>;

    ColumnKernel columnKernel(1, 2, 1);
    RowKernel rowKernel = columnKernel.transpose();

    Eigen::Matrix<TestType, 3, 3> kernel = columnKernel * rowKernel;

    Eigen::Matrix<TestType, 4, 4> input{
        {15, 50, 30, 20},
        {50, 55, 60, 30},
        {15, 65, 30, 15},
        {20, 30, 20, 25}};

    Eigen::Matrix<TestType, 4, 4> expected{
        {31, 41, 37, 26},
        {38, 48, 43, 29},
        {32, 42, 36, 25},
        {24, 30, 27, 23}};

    Eigen::Matrix<TestType, 6, 6> extended = tau::Extend(input, 1, 1);
    tau::Replicate(extended, 1, 1);

    auto twoD = tau::Convolve2d(kernel, extended);

    Eigen::Matrix<TestType, 4, 4> result =
        tau::Normalize(kernel, twoD)
            .block(1, 1, 4, 4)
            .array().round();

    REQUIRE(result == expected);

    Eigen::Matrix<TestType, 6, 6> separableInOut = tau::Extend(input, 1, 1);
    tau::Replicate(separableInOut, 1, 1);

    tau::ConvolveSeparable(
        tau::Kernel(rowKernel),
        tau::Kernel(columnKernel),
        separableInOut);

    Eigen::Matrix<TestType, 4, 4> separableResult =
        tau::Normalize(kernel, separableInOut).block(1, 1, 4, 4)
            .array().round();

    REQUIRE(separableResult == expected);

    Eigen::Matrix<TestType, 6, 6> separableInput = tau::Extend(input, 1, 1);
    tau::Replicate(separableInput, 1, 1);

    Eigen::Matrix<TestType, 6, 6> separableOutput;

    std::cout << "separableInput:\n" << separableInput << std::endl;

    tau::ConvolveSeparable(
        tau::Kernel(rowKernel),
        tau::Kernel(columnKernel),
        separableInput,
        separableOutput);

    std::cout << "separableOutput:\n" << separableOutput << std::endl;

    Eigen::Matrix<TestType, 4, 4> separableResult2 =
        tau::Normalize(kernel, separableOutput).block(1, 1, 4, 4)
            .array().round();

    REQUIRE(separableResult2 == expected);

    Eigen::Matrix<TestType, 6, 6> separableAliased = tau::Extend(input, 1, 1);
    tau::Replicate(separableAliased, 1, 1);

    tau::ConvolveSeparable(
        tau::Kernel(rowKernel),
        tau::Kernel(columnKernel),
        separableAliased);

    Eigen::Matrix<TestType, 4, 4> separableResult3 =
        tau::Normalize(kernel, separableAliased).block(1, 1, 4, 4)
            .array().round();

    REQUIRE(separableResult3 == expected);
}


TEMPLATE_TEST_CASE(
    "Margin choose largest row/column dimensions.",
    "[convolve]",
    int,
    float,
    double)
{
    using Type1 = Eigen::Matrix<TestType, 4, 6>;
    using Type2 = Eigen::Matrix<TestType, Eigen::Dynamic, 6>;
    using Type3 = Eigen::Matrix<TestType, Eigen::Dynamic, Eigen::Dynamic>;

    Type1 type1{};
    Type2 type2(14, 6);
    Type3 type3(2, 4);
    Type3 anotherType3(15, 7);

    auto margins = tau::Margins::Create(type1, type2, type3, anotherType3);

    REQUIRE(margins.horizontalMargin == 3);
    REQUIRE(margins.verticalMargin == 7);
}


TEST_CASE("Create an Eigen::Ref", "[convolve]")
{
    static constexpr auto D = Eigen::Dynamic;
    using M = Eigen::Matrix<int, D, D>;
    M data = M::Zero(4, 6);

    using R = Eigen::Ref<Eigen::Matrix<int, D, D>>;

    R r(data.block(0, 0, 3, 3));
    r(2, 2) = 14;

    REQUIRE(data(2, 2) == 14);
}


TEST_CASE("MakeView creates an Eigen::Ref", "[convolve]")
{
    static constexpr auto D = Eigen::Dynamic;
    using M = Eigen::Matrix<int, D, D>;
    M data = M::Zero(4, 6);
    auto view = tau::MakeView(data.block(1, 1, 3, 3));
    view(1, 1) = 14;
    REQUIRE(data(2, 2) == 14);
}


TEST_CASE("MakeView creates a const Eigen::Ref", "[convolve]")
{
    static constexpr auto D = Eigen::Dynamic;
    using M = Eigen::Matrix<int, D, D>;
    M data = M::Zero(4, 6);
    const M &p = data;

    auto view = tau::MakeView(p.block(1, 1, 3, 3));

    STATIC_REQUIRE(tau::IsEigenConstRef<decltype(view)>);
}


TEST_CASE("Can form a ConstView from non-const source", "[convolve]")
{
    static constexpr auto D = Eigen::Dynamic;
    using M = Eigen::Matrix<int, D, D>;
    M data = M::Zero(4, 6);
    auto view = tau::ConstView<M>(data);

    STATIC_REQUIRE(tau::IsEigenConstRef<decltype(view)>);
}
