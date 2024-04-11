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

    RowVector y = 100 * Eigen::sin(2 * x.array() - pi / 8);
    RowVector p = 900 * Eigen::cos(300 * x.array());
    RowVector q = 75 * Eigen::cos(101 * x.array());
    RowVector r = 50 * Eigen::cos(72 * x.array());

    for (auto [start, width, u]: {
            std::make_tuple(135, 357, &p),
            std::make_tuple(195, 212, &q),
            std::make_tuple(240, 250, &r)})
    {
        y(seqN(start, width)) += (*u)(seqN(start, width));
    }

    y(seqN(0, 150)).array() = 0;
    y.tail(length - 300).array() = 0;

    auto random = tau::UniformRandom<double>(seed, -5, 5);

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


Eigen::RowVector<double, Eigen::Dynamic> MakeWorstCaseTestSignal(
        [[maybe_unused]] tau::WaveletName name)
{
    using RowVector = Eigen::RowVector<double, Eigen::Dynamic>;
    using Eigen::Index;
    using Eigen::seqN;

    Index length = 1024;
    RowVector result = RowVector::Zero(length);

#if 1
    auto wavelet = tau::GetWavelet<double>(name);

    for (ssize_t i = 0; i < wavelet.decompose.low.size(); ++i)
    {
        result(seqN(350 + i, wavelet.decompose.low.size())).array()
            += wavelet.decompose.low.array();
    }

    for (ssize_t i = 0; i < wavelet.decompose.high.size(); ++i)
    {
        result(seqN(700 + i, wavelet.decompose.high.size())).array()
            += wavelet.decompose.high.array();
    }
#endif

    std::cout << "sum: " << wavelet.decompose.low.sum() << std::endl;
    std::cout << "size: " << wavelet.decompose.low.size() << std::endl;

    double range = result.maxCoeff() - result.minCoeff();
    result = result.array() - result.minCoeff();

    result = result.array() * 1023.0 / range;

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


TEMPLATE_TEST_CASE(
    "Wavelet decomposition maximum value",
    "[wavelet]",
    WaveletType<tau::WaveletName::db1>,
    WaveletType<tau::WaveletName::db4>,
    WaveletType<tau::WaveletName::db8>,
    WaveletType<tau::WaveletName::db16>,
    WaveletType<tau::WaveletName::db20>)
{
    auto seed = GENERATE(
        take(2, random(0u, std::numeric_limits<unsigned int>::max())));

    std::cout << '\n' << TestType::name << std::endl;
    auto signal = MakeTestSignal(seed);

    double maximum = std::max(std::abs(signal.maxCoeff()), std::abs(signal.minCoeff()));

    signal = signal.array() * 1023.0 / maximum;

    auto wavelet = tau::GetWavelet<double>(TestType::name);
    auto decomposed = tau::Decompose(wavelet, signal, true);

    size_t count = 0;

    for (auto &d: decomposed)
    {
        std::cout << count++ << ": "
            << std::max(std::abs(d.minCoeff()), std::abs(d.maxCoeff()))
            << std::endl;
    }

    auto recomposed = tau::Recompose(wavelet, decomposed, true);

    REQUIRE(recomposed.size() == signal.size());
    REQUIRE(signal.isApprox(recomposed));
}


TEMPLATE_TEST_CASE(
    "Worst-case coefficient amplitude",
    "[wavelet]",
    WaveletType<tau::WaveletName::db1>,
    WaveletType<tau::WaveletName::db2>,
    WaveletType<tau::WaveletName::db3>,
    WaveletType<tau::WaveletName::db4>,
    WaveletType<tau::WaveletName::db5>,
    WaveletType<tau::WaveletName::db6>,
    WaveletType<tau::WaveletName::db7>,
    WaveletType<tau::WaveletName::db8>,
    WaveletType<tau::WaveletName::db9>,
    WaveletType<tau::WaveletName::db10>,
    WaveletType<tau::WaveletName::db11>,
    WaveletType<tau::WaveletName::db12>,
    WaveletType<tau::WaveletName::db13>,
    WaveletType<tau::WaveletName::db14>,
    WaveletType<tau::WaveletName::db15>,
    WaveletType<tau::WaveletName::db16>,
    WaveletType<tau::WaveletName::db17>,
    WaveletType<tau::WaveletName::db18>,
    WaveletType<tau::WaveletName::db19>,
    WaveletType<tau::WaveletName::db20>)
{
    std::cout << "\nWorst Case\n" << TestType::name << std::endl;
    auto signal = MakeWorstCaseTestSignal(TestType::name);

    auto wavelet = tau::GetWavelet<double>(TestType::name);
    auto decomposed = tau::Decompose(wavelet, signal, true);

    // size_t count = 0;
    double maxCoeff = 0.0;

    for (auto &d: decomposed)
    {
        double thisMax =
            std::max(std::abs(d.minCoeff()), std::abs(d.maxCoeff()));

        maxCoeff = std::max(maxCoeff, thisMax);
    }

    std::cout << "maxCoeff: " << maxCoeff << std::endl;

    std::cout << "maxCoeff / size: "
        << maxCoeff / static_cast<double>(wavelet.decompose.low.size())
        << std::endl;

    auto recomposed = tau::Recompose(wavelet, decomposed, true);

    REQUIRE(recomposed.size() == signal.size());
    REQUIRE(signal.isApprox(recomposed));
}
