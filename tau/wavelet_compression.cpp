#include "tau/wavelet_compression.h"


namespace tau
{


uint8_t MoveSignBit(int8_t value, size_t width)
{
    int mask = (1 << (width - 1)) - 1;
    int signBit = (value & 0x80) >> (8 - width);

    return static_cast<uint8_t>(signBit | (value & mask));
}


int8_t ExtendSignBit(uint8_t value, size_t width)
{
    // Strip off other control bits.
    int result = value & ((1 << width) - 1);

    int signExtend = (1 << (width - 1));

    return static_cast<int8_t>((result ^ signExtend) - signExtend);
}


template<typename T>
bool Convertible(int64_t value)
{
    using Limits = std::numeric_limits<T>;

    return (value >= Limits::min() && value <= Limits::max());

}


void WriteValue(std::ostream &output, int64_t value)
{
    /*
    The first byte has a block flag, an extension bit, and the value in the
    last 6 bits if it fits. If the extension bit is set, this value does not
    fit in 6 bits. The value bits are used to specify the number of bytes
    used to represent the value.
    */

    if (value >= -32 && value <= 31)
    {
        // Leave block bit and extension bit set to 0
        jive::io::Write(output, MoveSignBit(static_cast<int8_t>(value), 6));
    }
    else if (Convertible<int8_t>(value))
    {
        jive::io::Write(output, static_cast<uint8_t>(0x41));
        jive::io::Write(output, static_cast<uint8_t>(value));
    }
    else if (Convertible<int16_t>(value))
    {
        jive::io::Write(output, static_cast<uint8_t>(0x42));
        jive::io::Write(output, static_cast<int16_t>(value));
    }
    else if (Convertible<int32_t>(value))
    {
        jive::io::Write(output, static_cast<uint8_t>(0x44));
        jive::io::Write(output, static_cast<int32_t>(value));
    }
    else
    {
        jive::io::Write(output, static_cast<uint8_t>(0x48));
        jive::io::Write(output, static_cast<int64_t>(value));
    }
}


int64_t ReadValue(uint8_t firstByte, std::istream &input)
{
    if (firstByte & 0x40)
    {
        // Multi-byte value

        uint8_t byteCount = firstByte & 0x3F;

        switch (byteCount)
        {
            case 1:
                return jive::io::Read<int8_t>(input);

            case 2:
                return jive::io::Read<int16_t>(input);

            case 4:
                return jive::io::Read<int32_t>(input);

            case 8:
                return jive::io::Read<int64_t>(input);

            default:
                throw std::runtime_error("Unsupported data type");
        }
    }
    else
    {
        // Not multiByte
        return ExtendSignBit(firstByte, 6);
    }
}


// 2 ** 14 - 1
static constexpr int multibyteMaximumZeroCount = 16383;


void EncodeRow(
    std::ostream &output,
    const Eigen::RowVector<double, Eigen::Dynamic> &row,
    bool enableMultibyteZeros)
{
    int zeroCount = -1;
    int maximumZeroCount;
    int singleByteMaximumZeroCount;

    if (enableMultibyteZeros)
    {
        maximumZeroCount = multibyteMaximumZeroCount;
        singleByteMaximumZeroCount = 127;
    }
    else
    {
        singleByteMaximumZeroCount = 127;
        maximumZeroCount = singleByteMaximumZeroCount;
    }

    for (auto value: row)
    {
        if (static_cast<int>(value) == 0)
        {
            if (zeroCount == -1)
            {
                // This is the first in a string of zeros
                zeroCount = 1;
                continue;
            }

            zeroCount += 1;

            if (zeroCount == maximumZeroCount)
            {
                if (enableMultibyteZeros)
                {
                    // We have a full block of zeros
                    jive::io::Write(output, static_cast<uint16_t>(0xFFFF));
                }
                else
                {
                    jive::io::Write(output, static_cast<uint8_t>(0xFF));
                }

                // Restart a new block of zeros
                zeroCount = -1;
            }

            continue;
        }
        else
        {
            if (zeroCount != -1)
            {
                if (enableMultibyteZeros)
                {
                    // We were in the middle of a block of zeros.
                    // Write out the count.
                    if (zeroCount <= singleByteMaximumZeroCount)
                    {
                        jive::io::Write(
                            output,
                            static_cast<uint8_t>(128 + zeroCount));
                    }
                    else
                    {
                        uint8_t highByte =
                            static_cast<uint8_t>(0x80 | (zeroCount / 128));

                        uint8_t lowByte =
                            static_cast<uint8_t>(0x80 | (zeroCount % 128));

                        jive::io::Write(output, highByte);
                        jive::io::Write(output, lowByte);
                    }
                }
                else
                {
                    jive::io::Write(
                        output,
                        static_cast<uint8_t>(128 + zeroCount));
                }

                zeroCount = -1;
            }

            // Write out the value
            WriteValue(output, static_cast<int64_t>(value));
        }
    }

    if (zeroCount != -1)
    {
        // We have a leftover block of zeros to write
        if (enableMultibyteZeros)
        {
            // We were in the middle of a block of zeros.
            // Write out the count.
            if (zeroCount <= singleByteMaximumZeroCount)
            {
                jive::io::Write(
                    output,
                    static_cast<uint8_t>(128 + zeroCount));
            }
            else
            {
                uint8_t highByte =
                    static_cast<uint8_t>(0x80 | (zeroCount / 128));

                uint8_t lowByte =
                    static_cast<uint8_t>(0x80 | (zeroCount % 128));

                jive::io::Write(output, highByte);
                jive::io::Write(output, lowByte);
            }
        }
        else
        {
            jive::io::Write(
                output,
                static_cast<uint8_t>(128 + zeroCount));
        }
    }
}


void Encode(
    std::ostream &output,
    const Decomposed<double> &decomposed,
    bool enableMultibyteZeros)
{
    size_t rowCount = decomposed.size();
    jive::io::Write(output, static_cast<uint8_t>(rowCount));

    for (auto &row: decomposed)
    {
        jive::io::Write(output, static_cast<uint16_t>(row.size()));
    }

    for (auto &row: decomposed)
    {
        EncodeRow(output, row, enableMultibyteZeros);
    }
}


uint16_t ReadZeros(
    uint8_t firstByte,
    std::istream &input,
    bool enableMultibyteZeros,
    size_t neededCount)
{
    assert(firstByte & 0x80);

    if (!enableMultibyteZeros)
    {
        return firstByte & 0x7F;
    }

    uint8_t firstByteMasked = firstByte & 0x7F;

    if (firstByteMasked == neededCount)
    {
        return firstByteMasked;
    }

    uint8_t secondByte = jive::io::Read<uint8_t>(input);
    
    if (secondByte & 0x80)
    {
        // This is a two byte zero block.
        return (firstByteMasked * 128) + (secondByte & 0x7F);
    }
    else
    {
        // Single byte zero block
        // Put back the byte we do not need.
        input.seekg(-1, std::ios::cur);

        return firstByteMasked;
    }
}


Eigen::RowVector<double, Eigen::Dynamic> DecodeRow(
    std::istream &input,
    uint16_t length,
    bool enableMultibyteZeros)
{
    using Eigen::Index;

    uint16_t decodedCount = 0;
    Eigen::RowVector<double, Eigen::Dynamic> row(static_cast<Index>(length));

    while (decodedCount < length)
    {
        uint8_t entry = jive::io::Read<uint8_t>(input);

        if (entry & 0x80)
        {
            // Block bit is set
            uint16_t zeroCount = ReadZeros(
                entry,
                input,
                enableMultibyteZeros,
                length - decodedCount);

            if (zeroCount + decodedCount > length)
            {
                input.seekg(-2, std::ios::cur);

                uint8_t n2 = jive::io::Read<uint8_t>(input);
                uint8_t n1 = jive::io::Read<uint8_t>(input);
                uint8_t p0 = jive::io::Read<uint8_t>(input);
                uint8_t p1 = jive::io::Read<uint8_t>(input);

                throw std::runtime_error("bad zerocount");
            }

            row(Eigen::seqN(decodedCount, zeroCount)).array() = 0;
            decodedCount += zeroCount;
        }
        else
        {
            row(decodedCount) = static_cast<double>(ReadValue(entry, input));
            decodedCount += 1;
        }
    }

    return row;
}


Decomposed<double> Decode(
    std::istream &input,
    bool enableMultibyteZeros)
{
    Decomposed<double> result;
    auto rowCount = jive::io::Read<uint8_t>(input);
    std::vector<uint16_t> rowLengths;

    for (uint8_t i = 0; i < rowCount; ++i)
    {
        rowLengths.push_back(jive::io::Read<uint16_t>(input));
    }

    for (auto length: rowLengths)
    {
        result.push_back(DecodeRow(input, length, enableMultibyteZeros));
    }

    return result;
}


double PreserveHighest(Decomposed<double> &decomposed, double keepRatio)
{
    Eigen::Index valueCount = 0;

    for (auto &row: decomposed)
    {
        valueCount += row.size();
    }

    auto sortedCount =
        static_cast<size_t>(keepRatio * static_cast<double>(valueCount));

    auto highest = SortHighest(sortedCount, decomposed);
    double threshold = std::max(1.0, *highest.begin());

    for (auto &row: decomposed)
    {
        row = (row.array().abs() < threshold).select(0, row);
    }

    return threshold;
}


double Quantize(
    Decomposed<double> &decomposed,
    double threshold,
    std::optional<QuantizeRange> range)
{
    double quantize = std::round(threshold);

    if (range)
    {
        quantize = std::min(range->maximum, quantize);
        quantize = std::max(range->minimum, quantize);
    }

    // Minimum quantize is 1.
    // Smaller values would increase the size of coefficients.
    quantize = std::max(quantize, 1.0);

    for (auto &row: decomposed)
    {
        row.array() = (row.array() / quantize).round();
    }

    return quantize;
}


} // end namespace tau
