#include <catch2/catch.hpp>

#include <jive/equal.h>
#include "tau/random.h"
#include "tau/horner.h"
#include <iostream>
#include <iomanip>


using Number = typename Eigen::VectorXd::Scalar;

static constexpr size_t imprecision = 10;

template<typename T>
struct About: jive::About<T, imprecision>
{
    About(T value): jive::About<T, imprecision>(value)
    {

    }
};


TEST_CASE("Compute single value of polynomial.", "[polynomial]")
{
    using namespace tau;

    // Run tests for 8 different polynomial degrees between 1 and 16.
    auto degree = GENERATE(take(8, random(1, 16)));

    // catch2 random returns the same set of seeds on every run.
    // Use these seeds to create random coefficients.

    auto seed = GENERATE(take(8, random(SeedLimits::min(), SeedLimits::max())));

    auto termCount = Index(degree + 1);
    Eigen::VectorXd coefficients(termCount);

    tau::UniformRandom<Number>{seed}(coefficients);

    // Compute the polynomial for 8 input values.
    double x = GENERATE(take(8, random(-1.0, 1.0)));

    double result = 0.0;

    for (auto index: jive::Range<Eigen::Index>(0, termCount))
    {
        result += coefficients(index) * std::pow(x, index);
    }

    SECTION("Coefficients as a column vector.")
    {
        auto polynomialResult = Horner(x, coefficients);
        REQUIRE(About(polynomialResult) == result);
    }

    SECTION("Coefficients as a row vector.")
    {
        auto polynomialResult = Horner(x, coefficients);
        REQUIRE(About(polynomialResult) == result);
    }
}


TEST_CASE("Compute polynomial from vectored input data.", "[polynomial]")
{
    using namespace tau;
    auto factors = std::vector<double>({0.0, 0.0, 1.0});
    auto inputs = std::vector<double>{{-1.0, 0.0, 1.0}};
    auto result = Horner(inputs, factors);

    REQUIRE(result.size() == 3);
    REQUIRE(jive::About(1.0) == result.at(0));
    REQUIRE(jive::About(0.0) == result.at(1));
    REQUIRE(jive::About(1.0) == result.at(2));
}


TEST_CASE("Compute polynomial from random vector input data.", "[polynomial]")
{
    using namespace tau;

    // Run tests for 8 different polynomial degrees between 1 and 16.
    auto degree = GENERATE(take(8, random(1, 16)));

    // catch2 random returns the same set of seeds on every run.
    // Use these seeds to create random coefficients.
    auto seed = GENERATE(take(8, random(SeedLimits::min(), SeedLimits::max())));

    auto termCount = Eigen::Index(degree + 1);
    Eigen::VectorXd coefficients(termCount);

    tau::UniformRandom<Number>{seed}(coefficients);

    static constexpr size_t count = 8;

    // Compute the polynomial for 8 input values.
    auto x = GENERATE(take(8, chunk(count, random(-2.0, 2.0))));

    Eigen::VectorXd independents =
        Eigen::VectorXd::Map(x.data(), Eigen::Index(count));

    REQUIRE(independents.size() == count);

    auto result = std::array<double, count>{};

    for (auto index: jive::Range<Eigen::Index>(0, count))
    {
        for (auto factorIndex: jive::Range<Eigen::Index>(0, termCount))
        {
            result[size_t(index)] +=
                coefficients(factorIndex)
                    * std::pow(independents(index), factorIndex);
        }
    }

    SECTION("Coefficients as a column vector.")
    {
        auto polynomialResult = Horner(independents, coefficients);

        for (auto index: jive::Range<Eigen::Index>(0, count))
        {
            REQUIRE(About(result[size_t(index)]) == polynomialResult(index));
        }
    }

    SECTION("Coefficients as a row vector.")
    {
        auto coefficientVector =
            Eigen::RowVectorXd::Map(
                coefficients.data(),
                termCount);

        auto polynomialResult = Horner(independents, coefficientVector);

        for (auto index: jive::Range<Eigen::Index>(0, count))
        {
            REQUIRE(About(result[size_t(index)]) == polynomialResult(index));
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


template<typename I, typename F, typename R>
void Check(
    const Eigen::DenseBase<I> &independents,
    const Eigen::DenseBase<F> &factors,
    const Eigen::DenseBase<R> &result)
{
    using T = typename I::Scalar;
    auto degree = factors.size() - 1;

    REQUIRE(result.rows() == independents.rows());
    REQUIRE(result.cols() == independents.cols());

    for (auto i: jive::Range<Eigen::Index>(0, result.rows()))
    {
        for (auto j: jive::Range<Eigen::Index>(0, result.cols()))
        {
            T check = 0;

            for (auto k: jive::Range<Eigen::Index>(0, degree + 1))
            {
                auto power = static_cast<T>(degree - k);

                check +=
                    factors(degree - k)
                    * std::pow(independents(i, j), power);
            }

            REQUIRE(result(i, j) == Approx(check));
        }
    }
}


TEMPLATE_TEST_CASE_SIG(
    "Independent as compile-time Matrix",
    "[polynomial]",
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

    auto seed = GENERATE(take(8, random(SeedLimits::min(), SeedLimits::max())));

    auto random = tau::UniformRandom<T>(seed, f(-1.0), f(1.0));
    random(independents);

    random.SetRange(f(-1000.0), f(1000.0));

    SECTION("Column vector coefficients")
    {
        Eigen::Vector<T, degree + 1> factors;
        random(factors);

        auto result = Horner(independents, factors);

        using Result = decltype(result);

        STATIC_REQUIRE(MatrixTraits<Result>::rows == rows);
        STATIC_REQUIRE(MatrixTraits<Result>::columns == columns);
        STATIC_REQUIRE(std::is_same_v<T, typename Result::Scalar>);

        Check(independents, factors, result);
    }

    SECTION("Row vector coefficients")
    {
        Eigen::RowVector<T, degree + 1> factors;
        random(factors);

        auto result = Horner(independents, factors);

        using Result = decltype(result);

        STATIC_REQUIRE(MatrixTraits<Result>::rows == rows);
        STATIC_REQUIRE(MatrixTraits<Result>::columns == columns);

        Check(independents, factors, result);
    }
}


TEST_CASE("Independent as vector." "[polynomial]")
{
    using namespace tau;

    auto seed = GENERATE(take(8, random(SeedLimits::min(), SeedLimits::max())));
    auto random = UniformRandom<double>(seed, -1000.0, 1000.0);
    Eigen::Vector<double, 8> factors;
    random(factors);
    random.SetRange(-1.0, 1.0);

    SECTION("Input as column vector")
    {
        Eigen::Vector<double, 10> input;
        random(input);
        auto result = Horner(input, factors);

        using resultTraits = MatrixTraits<decltype(result)>;

        STATIC_REQUIRE(resultTraits::isColumnVector);
        STATIC_REQUIRE(resultTraits::rows == 10);

        Check(input, factors, result);
    }

    SECTION("Input as row vector")
    {
        Eigen::RowVector<double, 10> input;
        random(input);
        auto result = Horner(input, factors);

        using resultTraits = MatrixTraits<decltype(result)>;

        STATIC_REQUIRE(resultTraits::isRowVector);
        STATIC_REQUIRE(resultTraits::columns == 10);

        Check(input, factors, result);
    }
}


TEMPLATE_TEST_CASE(
    "Horner polynomial evaluation.",
    "[polynomial]",
    double,
    float)
{
    using namespace tau;
    using T = TestType;

    auto f = Float<T>{};
    static constexpr Eigen::Index degree = 8;

    Eigen::Vector<T, 20> independents;

    auto seed = GENERATE(take(8, random(SeedLimits::min(), SeedLimits::max())));

    auto random = tau::UniformRandom<T>(seed, f(-1.0), f(1.0));
    random(independents);

    random.SetRange(f(-1000.0), f(1000.0));

    Eigen::Vector<T, degree + 1> factors;
    random(factors);

    auto result = Horner(independents, factors);
    Check(independents, factors, result);
}
