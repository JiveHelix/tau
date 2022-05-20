#include <catch2/catch.hpp>

#include "jive/range.h"
#include "turbo_matrix.h"
#include "tau/color_maps/turbo.h"
#include "tau/color_map.h"


TEST_CASE("Test standard turbo color map.", "[tau]")
{
    REQUIRE(turbo::standard.cols() == 3);
    REQUIRE(turbo::standard.rows() == 256);

    auto map = tau::turbo::MakeRgbFloat(256);

    // The power-series approximation of the turbo colormap is accurate
    // to within 0.0044155 of the standard value.
    REQUIRE((map - turbo::standard).array().abs().maxCoeff() < 0.0044155);
}


TEST_CASE("Test short floating-point turbo color map.", "[tau]")
{
    REQUIRE(turbo::shortMapFloat.cols() == 3);
    REQUIRE(turbo::shortMapFloat.rows() == 8);

    auto map = tau::turbo::MakeRgbFloat(8);

    for (auto row: jive::Range<Eigen::Index>(0, 8))
    {
        for (auto i: jive::Range<Eigen::Index>(0, 3))
        {
            // These test values were created from the power-series in numpy.
            // Expect them to be almost equal.
            auto standard = turbo::shortMapFloat(row, i);
            REQUIRE(map(row, i) == Approx(standard));
        }
    }
}


TEST_CASE("Test short RGB8 turbo color map.", "[tau]")
{
    REQUIRE(turbo::shortMapRgb8.cols() == 3);
    REQUIRE(turbo::shortMapRgb8.rows() == 8);

    auto map = tau::turbo::MakeRgb8(8);
    REQUIRE((map - turbo::shortMapRgb8).array().abs().maxCoeff() == 0);
    REQUIRE(map == turbo::shortMapRgb8);
}


TEST_CASE("Test indexing an RGB8 color map.", "[tau]")
{
    auto map = tau::turbo::MakeRgb8(8);

    Eigen::Matrix<int, 2, 3, Eigen::RowMajor> indices(
        {
            {7, 0, 4},
            {4, 7, 0}});

    auto mapped = 
        map(indices.reshaped<Eigen::AutoOrder>().array(), Eigen::all).eval();
    
    REQUIRE(mapped.rows() == indices.size());
    REQUIRE(mapped.cols() == 3);
}


TEST_CASE("Use ColorMap class to create RGB8 colors.", "[tau]")
{
    auto turbo = tau::turbo::MakeRgb8(8).eval();

    auto map = tau::ColorMap(turbo);

    Eigen::Matrix<int, 2, 3, Eigen::RowMajor> indices(
        {
            {7, 0, 4},
            {4, 7, 0}});

    Eigen::Matrix<uint8_t, 6, 3> mapped;

    map(indices, &mapped);
    
    Eigen::Matrix<uint8_t, 6, 3> expected;

    expected << 
        turbo(7, Eigen::all),
        turbo(0, Eigen::all),
        turbo(4, Eigen::all),
        turbo(4, Eigen::all),
        turbo(7, Eigen::all),
        turbo(0, Eigen::all);

    REQUIRE(mapped == expected);
}


TEST_CASE("Test Rescale.", "[tau]")
{
    using TestMatrix = Eigen::Matrix<int, 2, 3, Eigen::RowMajor>;

    TestMatrix test(
        {
            {2, 5, 8},
            {11, 14, 17}});
    
    auto rescale = tau::FloatRescale<int>(2, 17);
    tau::IndexMatrix<TestMatrix> indices = rescale(6, test);
    
    tau::IndexMatrix<TestMatrix> expected(
        {
            {0, 1, 2},
            {3, 4, 5}});
   
    REQUIRE(indices == expected);
}


TEST_CASE("Test ScaledColorMap.", "[tau]")
{
    using TestMatrix = Eigen::Matrix<int, 2, 4, Eigen::RowMajor>;

    TestMatrix test(
        {
            {2, 5, 8, 11},
            {14, 17, 20, 23}});

    auto turbo = tau::turbo::MakeRgb8(8);

    auto scaledColorMap = tau::ScaledColorMap(turbo, 2, 23);

    Eigen::Matrix<uint8_t, 8, 3> mapped;

    scaledColorMap(test, &mapped);
    
    Eigen::Matrix<uint8_t, 8, 3> expected;

    expected << 
        turbo(0, Eigen::all),
        turbo(1, Eigen::all),
        turbo(2, Eigen::all),
        turbo(3, Eigen::all),
        turbo(4, Eigen::all),
        turbo(5, Eigen::all),
        turbo(6, Eigen::all),
        turbo(7, Eigen::all);

    REQUIRE(mapped == expected);
}


TEST_CASE("Indexing of column-major matrix", "[tau]")
{
    using TestMatrix = Eigen::Matrix<int, 4, 2, Eigen::ColMajor>;
    TestMatrix test(
        {
            {0, 4},
            {1, 5},
            {2, 6},
            {3, 7}});
    
    auto second = test(Eigen::all, 1);
    int *columnData = second.data();

    for (int i = 0; i < 4; ++i)
    {
        REQUIRE(columnData[i] == i + 4);
    }
}


TEST_CASE("Indexing of row-major matrix", "[tau]")
{
    using TestMatrix = Eigen::Matrix<int, 2, 4, Eigen::RowMajor>;

    TestMatrix test(
        {
            {0, 1, 2, 3},
            {4, 5, 6, 7}});
    
    auto second = test(1, Eigen::all);
    int *rowData = second.data();

    for (int i = 0; i < 4; ++i)
    {
        REQUIRE(rowData[i] == i + 4);
    }
}


TEST_CASE("Unravel a matrix", "[tau]")
{
    using TestMatrix = Eigen::Matrix<int, 4, 3, Eigen::ColMajor>;

    TestMatrix test(
        {
            {0, 1, 2},
            {3, 4, 5},
            {6, 7, 8},
            {9, 10, 11}});

    Eigen::Matrix<int, 4, 3, Eigen::RowMajor> rowMajor = test;

    int *all = rowMajor.data();

    for (int i = 0; i < 12; ++i)
    {
        REQUIRE(all[i] == i);
    }
}
