#pragma once

#include <ostream>
#include <string>
#include <cmath>
#include <fields/fields.h>
#include "tau/row_convolve.h"
#include "tau/arithmetic.h"


namespace tau
{


enum class WaveletName
{
    db1,
    db2,
    db3,
    db4,
    db5,
    db6,
    db7,
    db8,
    db9,
    db10,
    db11,
    db12,
    db13,
    db14,
    db15,
    db16,
    db17,
    db18,
    db19,
    db20
};


std::string ToString(WaveletName name);


std::ostream & operator<<(std::ostream &outputStream, WaveletName name);


template<typename T>
struct WaveletFilter
{
    Eigen::RowVector<T, Eigen::Dynamic> low;
    Eigen::RowVector<T, Eigen::Dynamic> high;

    static constexpr auto fields = std::make_tuple(
        fields::Field(&WaveletFilter::low, "low"),
        fields::Field(&WaveletFilter::high, "high"));
};


template<typename T>
struct Wavelet
{
    WaveletName name;
    WaveletFilter<T> decompose;
    WaveletFilter<T> recompose;

    template<typename U, typename Style = Round>
    Wavelet<U> Cast() const
    {
        return CastFields<Wavelet<U>, U, Style>(*this);
    }

    size_t GetMaximumLevel(ssize_t signalSize) const
    {
        auto filterLength = static_cast<double>(this->decompose.low.size());
        assert(filterLength > 1);

        auto signalLength = static_cast<double>(signalSize);

        if (signalLength < filterLength - 1)
        {
            return 0;
        }

        return static_cast<size_t>(
            std::floor(
                std::log2(
                    signalLength / (filterLength - 1))));
    }

    ssize_t GetRecomposedSize(ssize_t signalLength) const
    {
        ssize_t result =
            2 * signalLength
            - this->recompose.low.size()
            + 2;

        assert(result > 0);

        return result;
    }

    static constexpr auto fields = std::make_tuple(
        fields::Field(&Wavelet::name, "name"),
        fields::Field(&Wavelet::decompose, "decompose"),
        fields::Field(&Wavelet::recompose, "recompose"));
};


namespace detail
{


Wavelet<double> GetWavelet(WaveletName name);


} // end namespace detail


template<typename T>
Wavelet<T> GetWavelet(WaveletName name)
{
    static_assert(std::is_floating_point_v<T>);
    if constexpr (std::is_same_v<T, double>)
    {
        return detail::GetWavelet(name);
    }
    else
    {
        return detail::GetWavelet(name).template Cast<T>();
    }
}


template<typename T>
using Decomposed =
    std::vector<Eigen::RowVector<T, Eigen::Dynamic>>;


template<typename T>
Decomposed<T> Decompose(
    const Wavelet<T> &wavelet,
    Eigen::RowVector<T, Eigen::Dynamic> signal,
    bool reflect = false,
    std::optional<size_t> level = {})
{
    size_t levelCount = wavelet.GetMaximumLevel(signal.size());

    if (level)
    {
        levelCount = std::min(*level, levelCount);
    }

    using RowVector = Eigen::RowVector<T, Eigen::Dynamic>;

    RowVector filterLow = wavelet.decompose.low.reverse();
    RowVector filterHigh = wavelet.decompose.high.reverse();

    RowVector lowPass;
    RowVector highPass;
    Decomposed<T> result;

    using Eigen::seqN;

    while (levelCount--)
    {
        lowPass = DoRowConvolve(signal, filterLow, reflect);
        highPass = DoRowConvolve(signal, filterHigh, reflect);
        signal = lowPass(seqN(1, lowPass.size() / 2, 2));
        result.push_back(highPass(seqN(1, highPass.size() / 2, 2)));
    }

    result.push_back(signal);
    std::reverse(std::begin(result), std::end(result));

    return result;
}


template<typename T>
Eigen::RowVector<T, Eigen::Dynamic> Recompose(
    const Wavelet<T> &wavelet,
    const Decomposed<T> &decomposed,
    bool reflect = false)
{
    if (decomposed.size() < 2)
    {
        throw std::runtime_error("decomposed is empty");
    }

    using RowVector = Eigen::RowVector<T, Eigen::Dynamic>;
    using Eigen::seqN;

    RowVector filterLow = wavelet.recompose.low.reverse();
    RowVector filterHigh = wavelet.recompose.high.reverse();

    RowVector approximation = decomposed[0];
    RowVector trimmed;

    for (size_t i = 1; i < decomposed.size(); ++i)
    {
        ssize_t count = approximation.size();

        RowVector upscaledApproximation = RowVector::Zero(count * 2);
        upscaledApproximation(seqN(0, count, 2)) = approximation;

        RowVector upscaledDetail = RowVector::Zero(count * 2);
        upscaledDetail(seqN(0, count, 2)) = decomposed[i];

        ssize_t convolutionSize = count * 2 + filterLow.size() - 1;
        ssize_t recomposedSize = wavelet.GetRecomposedSize(count);

        // Recomposed size may be recomputed below, but the trim start is not
        // affected.
        ssize_t trimLength = convolutionSize - recomposedSize;
        ssize_t start = trimLength / 2;


        if (i < decomposed.size() - 1)
        {
            // There is still another details vector to process.
            // Limit the recomposed size to the size of the next details
            // vector.
            recomposedSize = std::min(recomposedSize, decomposed[i + 1].size());
        }

        approximation =
            DoRowConvolve(upscaledApproximation, filterLow, reflect)
            + DoRowConvolve(upscaledDetail, filterHigh, reflect);

        trimmed = approximation(seqN(start, recomposedSize));
        approximation = trimmed;
    }

    return approximation;
}


} // end namespace tau
