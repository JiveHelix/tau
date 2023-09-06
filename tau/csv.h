#pragma


#include <string>
#include <fstream>
#include <unordered_map>
#include <jive/strings.h>
#include <jive/to_integer.h>
#include <jive/to_float.h>
#include <jive/precise_string.h>
#include "tau/size.h"
#include "tau/eigen_shim.h"


namespace tau
{


template<typename T, typename TokenT>
std::vector<T> ConvertLine(
    const std::string& input,
    const TokenT &token,
    int limit = -1)
{
    size_t tokenSize;

    if constexpr (std::is_same_v<TokenT, char>)
    {
        tokenSize = 1;
    }
    else
    {
        tokenSize = token.size();
    }

    std::string::size_type position = 0;

    if (tokenSize == 0)
    {
        throw std::invalid_argument("token must have non-zero length.");
    }

    std::vector<T> result;
    std::string::size_type tokenIndex{};

    for (int i = 0; i != limit; i++)
    {
        tokenIndex = input.find(token, position);

        if (tokenIndex == std::string::npos)
        {
            break;
        }

        if constexpr (std::is_floating_point_v<T>)
        {
            result.push_back(jive::ToFloat<T>(
                input.substr(position, tokenIndex - position)));
        }
        else
        {
            result.push_back(jive::ToInteger<T>(
                input.substr(position, tokenIndex - position)));
        }

        position = tokenIndex + tokenSize;
    }

    // Convert the final value
    if constexpr (std::is_floating_point_v<T>)
    {
        result.push_back(jive::ToFloat<T>(
            input.substr(position, tokenIndex - position)));
    }
    else
    {
        result.push_back(jive::ToInteger<T>(
            input.substr(position, tokenIndex - position)));
    }

    return result;
}


template<typename T, int options = Eigen::ColMajor>
class Csv
{
public:
    using Index = typename Eigen::Index;
    using Size = tau::Size<Index>;
    using Data = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, options>;

    Csv(
        const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, options> &data,
        const std::vector<std::string> &headers = {})
        :
        headers_(headers),
        headerMap_(),
        data_(data)
    {
        // Store in unordered_map to decrease search time when looking up
        // a column by header name.
        Index columnIndex = 0;

        for (auto &header: this->headers_)
        {
            // Clean up whitespace
            header = jive::strings::Trim(header);
            this->headerMap_[header] = columnIndex++;
        }
    }

    Csv(const std::string &fileName, bool hasHeaders)
    {
        std::ifstream inputStream(fileName);

        if (!inputStream)
        {
            throw std::runtime_error("Unable to open file: " + fileName);
        }

        std::string line;

        uint8_t bom[4]{};

        inputStream.read(reinterpret_cast<char *>(&bom[0]), 4);

        if (!inputStream)
        {
            throw std::runtime_error("Unable to read file: " + fileName);
        }

        if (
            (bom[0] == 0xFE)
            || (bom[0] == 0xFF)
            || (bom[0] == 0))
        {
            throw std::runtime_error(
                "Wide encodings not supported. Recreate the csv as utf8.");
        }

        if ((bom[0] == 0xEF) && (bom[1] == 0xBB) && (bom[2] == 0xBF))
        {
            // UTF8 encoding.
            inputStream.seekg(3);
        }
        else
        {
            // Treat the file as if there is no BOM.
            inputStream.seekg(0);
        }

        if (hasHeaders)
        {
            std::getline(inputStream, line, '\n');

            if (!inputStream)
            {
                throw std::runtime_error("File appears to be empty");
            }

            this->headers_ = jive::strings::Split(line, ',');

            // Store in unordered_map to decrease search time when looking up
            // a column by header name.
            Index columnIndex = 0;

            for (auto &header: this->headers_)
            {
                // Clean up whitespace
                header = jive::strings::Trim(header);
                this->headerMap_[header] = columnIndex++;
            }
        }

        std::vector<std::vector<T>> inputs;
        size_t maximumColumnCount = 0;

        while (true)
        {
            std::getline(inputStream, line, '\n');

            if (!inputStream)
            {
                break;
            }

            inputs.push_back(ConvertLine<T>(line, ','));

            maximumColumnCount =
                std::max(maximumColumnCount, inputs.back().size());
        }

        if (maximumColumnCount == 0)
        {
            throw std::runtime_error("File has no data");
        }

        Index rowCount = inputs.size();

        this->data_ = Data::Zero(rowCount, maximumColumnCount);

        for (Index row = 0; row < rowCount; ++row)
        {
            auto &values = inputs[static_cast<size_t>(row)];

            for (
                Index column = 0;
                column < static_cast<Index>(values.size());
                ++column)
            {
                this->data_(row, column) = values[column];
            }
        }
    }

    const std::vector<std::string> & GetHeaders() const
    {
        return this->headers_;
    }

    Size GetSize() const
    {
        return {this->data_.cols(), this->data_.rows()};
    }

    T operator()(Index row, Index column) const
    {
        return this->data_(row, column);
    }

    T operator()(const std::string &headerName, Index row) const
    {
        return this->data_(row, this->headerMap_.at(headerName));
    }

    Data GetData() const
    {
        return this->data_;
    }

    void ToFile(const std::string &fileName)
    {
        std::ofstream outputStream(fileName);

        if (!outputStream)
        {
            throw std::runtime_error(
                "Cannot open " + fileName + " for writing");
        }

        this->ToStream(outputStream);
    }

    std::ostream & ToStream(std::ostream & outputStream)
    {
        if (!this->headers_.empty())
        {
            outputStream <<
                jive::strings::Join(
                    std::begin(this->headers_),
                    std::end(this->headers_),
                    ',') << std::endl;
        }

        Index rowCount = this->data_.rows();
        Index columnCount = this->data_.cols();

        for (Index row = 0; row < rowCount; ++row)
        {
            for (Index column = 0; column < columnCount - 1; ++column)
            {
                outputStream
                    << jive::PreciseString(this->data_(row, column))
                    << ",";
            }

            // Append the last value without a comma.
            outputStream << jive::PreciseString(
                this->data_(row, columnCount - 1));

            outputStream << '\n';
        }

        return outputStream;
    }

private:
    std::vector<std::string> headers_;
    std::unordered_map<std::string, Index> headerMap_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, options> data_;
};


} // end namespace tau
