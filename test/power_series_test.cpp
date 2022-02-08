#include <catch2/catch.hpp>
#include <limits>
#include <random>
#include "tau/power_series.h"
#include "Eigen/Dense"
#include "jive/range.h"


using seedLimits = std::numeric_limits<unsigned int>;
using Index = Eigen::Index;


struct Random
{
public:
    Random(unsigned int seed): generator_(seed)
    {

    }

    template<typename Matrix, typename Range = double>
    void operator()(
        Matrix &matrix,
        Range low = -1000.0,
        Range high = 1000.0)
    {
        using Scalar = typename tau::MatrixTraits<Matrix>::type;

        std::uniform_real_distribution<Scalar> distribution(
            static_cast<Scalar>(low),
            static_cast<Scalar>(high));

        for (auto i: jive::Range<Index>(0, matrix.rows()))
        {
            for (auto j: jive::Range<Index>(0, matrix.cols()))
            {
                matrix(i, j) = distribution(this->generator_);
            }
        }
    }

private:
    std::mt19937 generator_;
};


TEST_CASE("Compute single value from power series.", "[power_series]")
{
    using namespace tau;

    // Run tests for 8 different power series degrees between 1 and 16.
    auto degree = GENERATE(take(8, random(1, 16)));

    // catch2 random returns the same set of seeds on every run.
    // Use these seeds to create random coefficients.

    auto seed = GENERATE(take(8, random(seedLimits::min(), seedLimits::max())));
    
    auto termCount = static_cast<Index>(degree + 1);
    Eigen::VectorXd coefficients(termCount);

    Random{seed}(coefficients);
    
    // Compute the power series for 8 input values.
    double x = GENERATE(take(8, random(-1.0, 1.0)));

    double result = 0.0;

    for (auto index: jive::Range<Index>(0, termCount))
    {
        result += coefficients(index) * std::pow(x, index);
    }

    SECTION("Coefficients as a column vector.")
    {
        auto powerSeriesResult = 
            tau::PowerSeries(x, coefficients, LinearMap<double>{});

        REQUIRE(result == Approx(powerSeriesResult));
    }

    SECTION("Coefficients as a row vector.")
    {
        auto powerSeriesResult = 
            tau::PowerSeries(x, coefficients, LinearMap<double>{});

        REQUIRE(result == Approx(powerSeriesResult));
    }
}


TEST_CASE("Apply power series to vectored input data.", "[power_series]")
{
    using namespace tau;
    auto factors = Vector<3, double>(0.0, 0.0, 1.0);
    auto inputs = std::vector<double>{{-1.0, 0.0, 1.0}};
    auto result = PowerSeries(inputs, factors, LinearMap<double>{});
    
    REQUIRE(result.size() == 3);
    REQUIRE(result(0) == Approx(1.0));
    REQUIRE(result(1) == Approx(0.0));
    REQUIRE(result(2) == Approx(1.0));
}


TEST_CASE("Apply power series to random vectored input data.", "[power_series]")
{
    using namespace tau;

    // Run tests for 8 different power series degrees between 1 and 16.
    auto degree = GENERATE(take(8, random(1, 16)));

    // catch2 random returns the same set of seeds on every run.
    // Use these seeds to create random coefficients.
    auto seed = GENERATE(take(8, random(seedLimits::min(), seedLimits::max())));
    
    auto termCount = static_cast<Index>(degree + 1);
    Eigen::VectorXd coefficients(termCount);
    Random{seed}(coefficients);

    static constexpr size_t count = 8;
    
    // Compute the power series for 8 input values.
    auto x = GENERATE(take(8, chunk(count, random(-2.0, 2.0))));

    auto result = std::array<double, count>{};

    for (auto index: jive::Range<size_t>(0, count))
    {
        for (auto factorIndex: jive::Range<Index>(0, termCount))
        {
            result[index] +=
                coefficients(factorIndex) * std::pow(x[index], factorIndex);
        }
    }

    SECTION("Coefficients as a column vector.")
    {
        auto powerSeriesResult = 
            tau::PowerSeries(x, coefficients, LinearMap<double>{});

        for (auto index: jive::Range<Index>(0, count))
        {
            REQUIRE(
                result[static_cast<size_t>(index)]
                == Approx(powerSeriesResult(index)));
        }
    }

    SECTION("Coefficients as a row vector.")
    {
        auto coefficientVector =
            Eigen::RowVectorXd::Map(
                coefficients.data(),
                static_cast<unsigned>(termCount));

        auto powerSeriesResult = 
            tau::PowerSeries(x, coefficientVector, LinearMap<double>{});

        // REQUIRE(result == Approx(powerSeriesResult));

        for (auto index: jive::Range<Index>(0, count))
        {
            REQUIRE(
                result[static_cast<size_t>(index)]
                == Approx(powerSeriesResult(index)));
        }
    }
}


template<typename T>
struct Float
{
    template<typename U>
    T operator()(U &&value)
    {
        return static_cast<T>(value);
    }
};


TEMPLATE_TEST_CASE_SIG(
    "Independent as compile-time Matrix",
    "[power_series]",
    ((typename T, int rows, int columns, int degree), T, rows, columns, degree),
    (double, 1, 8, 5),
    (double, 8, 1, 6),
    (double, 4, 6, 7),
    (double, 6, 4, 8),
    (float, 1, 8, 7),
    (float, 8, 1, 6),
    (float, 4, 6, 5),
    (float, 6, 4, 4))
{
    using namespace tau;
    
    auto f = Float<T>{};

    Eigen::Matrix<T, rows, columns> independents;

    auto seed = GENERATE(take(8, random(seedLimits::min(), seedLimits::max())));

    auto random = Random(seed);
    random(independents, f(-1.0), f(1.0));

    SECTION("Column vector coefficients")
    {
        Eigen::Vector<T, degree + 1> factors;
        random(factors);

        auto powerSeriesResult =
            PowerSeries(independents, factors, LinearMap<T>{});
        
        using Result = decltype(powerSeriesResult);

        STATIC_REQUIRE(MatrixTraits<Result>::rows == rows);
        STATIC_REQUIRE(MatrixTraits<Result>::columns == columns);

        for (auto i: jive::Range<Index>(0, rows))
        {
            for (auto j: jive::Range<Index>(0, columns))
            {
                T check = 0;

                for (auto k: jive::Range<Index>(0, degree + 1))
                {    
                    auto power = static_cast<T>(k);
                    check += factors(k) * std::pow(independents(i, j), power);
                }

                REQUIRE(powerSeriesResult(i, j) == Approx(check));
            }
        }
    }

    SECTION("Row vector coefficients")
    {
        Eigen::RowVector<T, degree + 1> factors;
        random(factors);

        auto powerSeriesResult =
            PowerSeries(independents, factors, LinearMap<T>{});
        
        using Result = decltype(powerSeriesResult);

        STATIC_REQUIRE(MatrixTraits<Result>::rows == rows);
        STATIC_REQUIRE(MatrixTraits<Result>::columns == columns);

        for (auto i: jive::Range<Index>(0, rows))
        {
            for (auto j: jive::Range<Index>(0, columns))
            {
                T check = 0;

                for (auto k: jive::Range<Index>(0, degree + 1))
                {    
                    auto power = static_cast<T>(k);
                    check += factors(k) * std::pow(independents(i, j), power);
                }

                REQUIRE(powerSeriesResult(i, j) == Approx(check));
            }
        }
    }
}


TEST_CASE("Independent as vector." "[power_series]")
{
    using namespace tau;

    Eigen::Vector<double, 8> factors;

    SECTION("Input as column vector")
    {
        Eigen::Vector<double, 10> input;
        auto result = PowerSeries(input, factors, LinearMap<double>{});

        using resultTraits = MatrixTraits<decltype(result)>;

        STATIC_REQUIRE(resultTraits::isColumnVector);
        STATIC_REQUIRE(resultTraits::rows == 10);
    }

    SECTION("Input as row vector")
    {
        Eigen::RowVector<double, 10> input;
        auto result = PowerSeries(input, factors, LinearMap<double>{});

        using resultTraits = MatrixTraits<decltype(result)>;

        STATIC_REQUIRE(resultTraits::isRowVector);
        STATIC_REQUIRE(resultTraits::columns == 10);
    }
}
