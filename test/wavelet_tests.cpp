#include <catch2/catch.hpp>
#include <tau/wavelet.h>
#include <tau/random.h>
#include <tau/angles.h>


Eigen::RowVector<double, Eigen::Dynamic> MakeTestSignal(tau::Seed seed)
{
    using RowVector = Eigen::RowVector<double, Eigen::Dynamic>;
    using Eigen::Index;
    using Eigen::seqN;

    Index length = 1024;
    auto pi = tau::Angles<double>::pi;
    RowVector x = RowVector::LinSpaced(length, 0, 2 * pi);

    RowVector y = 175 * Eigen::sin(2 * x.array() - pi / 8);
    RowVector p = 7 * Eigen::cos(120 * x.array());
    RowVector q = 9 * Eigen::cos(101 * x.array());
    RowVector r = 5 * Eigen::cos(72 * x.array());

    for (auto [start, width, u]: {
            std::make_tuple(135, 357, &p),
            std::make_tuple(195, 212, &q),
            std::make_tuple(240, 250, &r)})
    {
        y(seqN(start, width)) += (*u)(seqN(start, width));
    }

    y(seqN(0, 150)).array() = 0;
    y.tail(length - 300).array() = 0;

    auto random = tau::UniformRandom<double>(seed, -0.5, 0.5);

    for (ssize_t i = 0; i < y.size(); ++i)
    {
        y(i) += random();
    }

    return y;
}


Eigen::RowVector<double, Eigen::Dynamic> MakeSimpleTestSignal()
{
    using RowVector = Eigen::RowVector<double, Eigen::Dynamic>;
    RowVector result{{0, 1, 2, 3, 4, 5, 4, 3, 2, 1}};

    return result;
}


TEST_CASE("Wavelet decompose with simple signal", "[wavelet]")
{
    auto signal = MakeSimpleTestSignal();
    auto wavelet = tau::GetWavelet<double>(tau::WaveletName::db1);
    auto decomposed = tau::Decompose(wavelet, signal);
    auto recomposed = tau::Recompose(wavelet, decomposed);

    REQUIRE(recomposed.size() == signal.size());
    REQUIRE(signal.isApprox(recomposed));
}


template<tau::WaveletName name_>
struct WaveletType
{
    static constexpr auto name = name_;
};


TEMPLATE_TEST_CASE(
    "Wavelet decompose round trip",
    "[wavelet]",
    WaveletType<tau::WaveletName::db1>,
    WaveletType<tau::WaveletName::db2>,
    WaveletType<tau::WaveletName::db4>,
    WaveletType<tau::WaveletName::db10>,
    WaveletType<tau::WaveletName::db12>,
    WaveletType<tau::WaveletName::db20>)
{
    auto seed = GENERATE(
        take(10, random(0u, std::numeric_limits<unsigned int>::max())));

    auto signal = MakeTestSignal(seed);

    auto wavelet = tau::GetWavelet<double>(TestType::name);
    auto decomposed = tau::Decompose(wavelet, signal);
    auto recomposed = tau::Recompose(wavelet, decomposed);

    REQUIRE(recomposed.size() == signal.size());
    REQUIRE(signal.isApprox(recomposed));
}


TEMPLATE_TEST_CASE(
    "Wavelet decompose with reflected extension",
    "[wavelet]",
    WaveletType<tau::WaveletName::db1>,
    WaveletType<tau::WaveletName::db2>,
    WaveletType<tau::WaveletName::db4>,
    WaveletType<tau::WaveletName::db10>,
    WaveletType<tau::WaveletName::db12>,
    WaveletType<tau::WaveletName::db20>)
{
    auto seed = GENERATE(
        take(10, random(0u, std::numeric_limits<unsigned int>::max())));

    auto signal = MakeTestSignal(seed);

    auto wavelet = tau::GetWavelet<double>(TestType::name);
    auto decomposed = tau::Decompose(wavelet, signal, true);
    auto recomposed = tau::Recompose(wavelet, decomposed, true);

    REQUIRE(recomposed.size() == signal.size());
    REQUIRE(signal.isApprox(recomposed));
}
