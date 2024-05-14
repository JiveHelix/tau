#pragma once


#include <istream>
#include <ostream>
#include <jive/binary_io.h>
#include <numeric>
#include <set>
#include <tau/eigen_shim.h>

#include "tau/wavelet.h"


namespace tau
{

template<typename T>
std::multiset<T> SortHighest(size_t count, const Decomposed<T> &decomposed)
{
    std::multiset<double> highest;

    for (const auto &row: decomposed)
    {
        for (auto value: row)
        {
            auto absValue = std::abs(value);

            if (highest.size() < count)
            {
                highest.insert(absValue);
                continue;
            }

            if (absValue > *highest.begin())
            {
                auto insertion = highest.erase(highest.begin());
                highest.insert(insertion, absValue);
            }
        }
    }

    return highest;
}


uint8_t MoveSignBit(int8_t value, size_t width);


int8_t ExtendSignBit(uint8_t value, size_t width);


void WriteValue(std::ostream &output, int64_t value);


int64_t ReadValue(uint8_t firstByte, std::istream &input);


void EncodeRow(
    std::ostream &output,
    const Eigen::RowVector<double, Eigen::Dynamic> &row,
    bool enableMultibyteZeros);


void Encode(
    std::ostream &output,
    const tau::Decomposed<double> &decomposed,
    bool enableMultibyteZeros);


Eigen::RowVector<double, Eigen::Dynamic> DecodeRow(
    std::istream &input,
    uint16_t length,
    bool enableMultibyteZeros);


tau::Decomposed<double> Decode(std::istream &input, bool enableMultibyteZeros);


double PreserveHighest(Decomposed<double> &decomposed, double keepRatio);


struct QuantizeRange
{
    double minimum;
    double maximum;
};


double Quantize(
    Decomposed<double> &decomposed,
    double threshold,
    std::optional<QuantizeRange> range = {});


template<typename Derived>
double GetRms(const Eigen::MatrixBase<Derived> &values)
{
    return std::sqrt(
        values.array().pow(2.0).sum() / static_cast<double>(values.size()));
}


} // end namespace tau
